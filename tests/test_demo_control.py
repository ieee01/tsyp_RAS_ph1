import pytest
import asyncio
import httpx

from living_map_command_post.demo import DemoController, DemoError
from living_map_command_post.state import CommandPostState
from living_map_command_post.web import create_app


def setup_demo():
    state = CommandPostState()
    state.replace({"robots": {"writer": {"state": "PAUSED"}, "executor": {"state": "READY"}},
                   "memories": [], "demo": {}})
    sent = []
    now = [0.0]
    demo = DemoController(state, sent.append, clock=lambda: now[0])
    return state, sent, now, demo


def ack(state, sent):
    snapshot = state.snapshot()
    snapshot["demo"] = {"ack": {"request_id": sent[-1]["request_id"], "success": True, "message": "OK"}}
    state.replace(snapshot)


def discoveries(state, types=(1, 2, 4)):
    snapshot = state.snapshot()
    snapshot["memories"] = [{"mission_id": 1, "beacon_id": 10 + kind, "event_type": kind, "freshness": "FRESH",
                             "severity": 3, "confidence": 0.9} for kind in types]
    if 1 in types:  # the default scenario has a second, less certain victim
        snapshot["memories"].append({"mission_id": 1, "beacon_id": 21, "event_type": 1, "freshness": "FRESH",
                                     "severity": 3, "confidence": 0.8})
    state.replace(snapshot)


def test_full_demo_waits_for_all_events_failure_and_real_completion():
    state, sent, now, demo = setup_demo()
    demo.request("full_demo")
    assert sent[-1]["action"] == "start"
    ack(state, sent)
    demo.tick()
    discoveries(state, (1,))
    demo.tick()
    assert len(sent) == 1  # A victim alone cannot pass the event requirement.
    discoveries(state, (1, 2))
    demo.tick()
    assert len(sent) == 1  # Fire must also be recorded before failing the Writer.
    discoveries(state)
    demo.tick()
    assert sent[-1]["action"] == "fail_writer"
    ack(state, sent)
    demo.tick()
    demo.tick()
    assert len(sent) == 2  # A service ack alone is not failure evidence.
    snapshot = state.snapshot()
    snapshot["robots"]["writer"]["state"] = "FAILED"
    state.replace(snapshot)
    demo.tick()
    assert sent[-1]["action"] == "dispatch"
    # WHO and WHAT only: the robot owns its route and hazard policy.
    assert sent[-1]["robot_id"] == "executor"
    assert sent[-1]["target_beacon_id"] == 11
    assert "avoid_event_types" not in sent[-1]
    ack(state, sent)
    demo.tick()
    assert demo.phase == "EXECUTING"
    snapshot = state.snapshot()
    snapshot["robots"]["executor"]["state"] = "MISSION_COMPLETE"
    state.replace(snapshot)
    demo.tick()
    assert demo.phase == "COMPLETE"
    assert not demo.automatic


def test_full_demo_waits_for_every_configured_gas_hazard():
    state, sent, now, _ = setup_demo()
    demo = DemoController(state, sent.append, clock=lambda: now[0], min_gas_hazards=2)
    demo.request("full_demo")
    ack(state, sent)
    demo.tick()
    discoveries(state)  # both victims, fire and one gas hazard
    demo.tick()
    assert sent[-1]["action"] == "start"  # the second gas hazard is still unrecorded
    snapshot = state.snapshot()
    snapshot["memories"].append({"mission_id": 1, "beacon_id": 30, "event_type": 2, "freshness": "FRESH",
                                 "severity": 4, "confidence": 0.94})
    state.replace(snapshot)
    demo.tick()
    assert sent[-1]["action"] == "fail_writer"
    assert DemoController.has_events(state.snapshot(), 2)
    assert not DemoController.has_events(state.snapshot(), 3)


def test_uplink_retry_preserves_command_id_and_expires():
    state, sent, now, demo = setup_demo()
    demo.request("start")
    for i in range(1, 6):
        now[0] = i * 9
        demo.tick()
    assert len(sent) == 5
    assert len({p["request_id"] for p in sent}) == 1
    assert demo.phase == "ERROR"
    assert demo.pending is None


def test_expired_victim_memory_cannot_enable_dispatch():
    state, sent, now, demo = setup_demo()
    discoveries(state)
    snapshot = state.snapshot()
    snapshot["robots"]["writer"]["state"] = "FAILED"
    for memory in snapshot["memories"]:
        if memory["event_type"] == 1:
            memory["freshness"] = "EXPIRED"
    state.replace(snapshot)
    with pytest.raises(DemoError, match="victim"):
        demo.request("dispatch")
    assert sent == []


def test_manual_dispatch_needs_only_victim_and_names_who_and_what():
    state, sent, now, demo = setup_demo()
    discoveries(state, (1, 4))  # Writer failed before reaching the gas chamber.
    snapshot = state.snapshot()
    snapshot["robots"]["writer"]["state"] = "FAILED"
    state.replace(snapshot)
    demo.request("dispatch")
    assert sent[-1]["action"] == "dispatch"
    assert (sent[-1]["robot_id"], sent[-1]["target_beacon_id"]) == ("executor", 11)
    assert any("WHO = executor" in event["message"] for event in demo.events)


def test_unknown_or_stale_telemetry_rejects_commands():
    state = CommandPostState()
    demo = DemoController(state, lambda p: None)
    with pytest.raises(DemoError, match="fresh"):
        demo.request("start")
    state.replace({"robots": {"writer": {"state": "PAUSED"}}, "memories": []})
    state.received_at -= 10
    with pytest.raises(DemoError, match="fresh"):
        demo.request("start")


def test_manual_resume_and_pause_and_failed_writer_guard():
    state, sent, now, demo = setup_demo()
    demo.request("start")
    ack(state, sent)
    demo.tick()
    demo.request("pause")
    assert sent[-1]["action"] == "pause"
    ack(state, sent)
    demo.tick()
    assert demo.phase == "PAUSED"
    snapshot = state.snapshot()
    snapshot["robots"]["writer"]["state"] = "FAILED"
    state.replace(snapshot)
    with pytest.raises(DemoError, match="restart"):
        demo.request("start")


def test_api_validates_actions_and_reports_rejected_commands():
    state, sent, now, demo = setup_demo()
    restarts = []
    app = create_app(state, lambda p: True, demo,
                     lambda mode, speed: restarts.append((mode, speed)) or mode == "guided")
    async def exercise():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
            assert (await client.post("/api/demo", json={"action": "not-a-command"})).status_code == 422
            assert (await client.post("/api/demo", json={"action": "dispatch"})).status_code == 409
            assert (await client.post("/api/demo", json={"action": "full_demo"})).json()["accepted"]
            assert (await client.get("/api/state")).json()["demo_control"]["automatic"]
            assert (await client.post("/api/demo/restart", json={"mode": "guided"})).status_code == 200
            assert (await client.post("/api/demo/restart", json={"mode": "invalid"})).status_code == 422
            assert (await client.post("/api/demo/restart", json={"mode": "guided", "speed": "very_fast"})).status_code == 200
            assert (await client.post("/api/demo/restart", json={"mode": "guided", "speed": "warp"})).status_code == 422
    asyncio.run(exercise())
    assert restarts == [("guided", "normal"), ("guided", "very_fast")]


def set_writer(state, value):
    snapshot = state.snapshot()
    snapshot["robots"]["writer"]["state"] = value
    state.replace(snapshot)


def test_stuck_outcome_immobilises_then_dispatches_on_stuck_telemetry():
    state, sent, now, demo = setup_demo()
    demo.request("full_demo", outcome="stuck")
    ack(state, sent)
    demo.tick()
    discoveries(state)
    demo.tick()
    assert sent[-1]["action"] == "stuck_writer"
    ack(state, sent)
    demo.tick()
    assert demo.phase == "WAITING_STUCK" and sent[-1]["action"] == "stuck_writer"
    set_writer(state, "STUCK")
    demo.tick()
    assert sent[-1]["action"] == "dispatch"


def test_return_outcome_waits_for_home_not_for_events():
    state, sent, now, demo = setup_demo()
    demo.request("full_demo", outcome="return")
    ack(state, sent)
    demo.tick()
    discoveries(state)
    demo.tick()
    assert len(sent) == 1  # all events recorded, but the Writer keeps mapping
    set_writer(state, "RETURNING")
    demo.tick()
    assert any("returning" in event["message"] for event in demo.events)
    set_writer(state, "HOME")
    demo.tick()
    assert demo.phase == "MEMORY_PRESERVED" and sent[-1]["action"] == "dispatch"


def test_writer_reporting_itself_stuck_hands_over_automatically():
    state, sent, now, demo = setup_demo()
    demo.request("full_demo")
    ack(state, sent)
    demo.tick()
    discoveries(state, (1,))
    set_writer(state, "STUCK")
    demo.tick()
    assert sent[-1]["action"] == "dispatch"


def test_dispatch_refused_while_writer_is_still_exploring():
    state, sent, now, demo = setup_demo()
    discoveries(state)
    set_writer(state, "ONLINE")
    with pytest.raises(DemoError, match="sortie"):
        demo.request("dispatch")


def test_beacon_can_be_destroyed_mid_sequence_but_not_over_a_pending_command():
    state, sent, now, demo = setup_demo()
    demo.request("full_demo")
    with pytest.raises(DemoError, match="acknowledge"):
        demo.request("destroy_beacon", beacon_id=3)
    ack(state, sent)
    demo.tick()
    demo.request("destroy_beacon", beacon_id=3)
    assert sent[-1] == {**sent[-1], "action": "destroy_beacon", "beacon_id": 3}
    with pytest.raises(DemoError):
        demo.request("start")


def test_unknown_outcome_is_rejected():
    state, sent, now, demo = setup_demo()
    with pytest.raises(DemoError, match="outcome"):
        demo.request("full_demo", outcome="teleport")


def test_dispatch_picks_most_urgent_victim():
    from living_map_command_post.demo import select_target
    snapshot = {"memories": [
        {"mission_id": 1, "beacon_id": 4, "event_type": 1, "severity": 2, "confidence": 0.99, "freshness": "FRESH"},
        {"mission_id": 1, "beacon_id": 9, "event_type": 1, "severity": 4, "confidence": 0.80, "freshness": "AGING"},
        {"mission_id": 1, "beacon_id": 7, "event_type": 1, "severity": 5, "confidence": 0.90, "freshness": "EXPIRED"},
    ]}
    assert select_target(snapshot)["beacon_id"] == 9


def fleet_demo():
    state, sent, now, demo = setup_demo()
    snapshot = state.snapshot()
    snapshot["robots"]["firebot"] = {"state": "READY"}
    state.replace(snapshot)
    return state, sent, now, demo


def test_command_post_sends_the_right_robot_to_each_event():
    state, sent, now, demo = fleet_demo()
    discoveries(state)
    set_writer(state, "FAILED")
    demo.request("dispatch")
    # Fire first, to the fire robot; the rescue robot waits until the scene is secured.
    assert (sent[-1]["robot_id"], sent[-1]["target_beacon_ids"]) == ("firebot", [14])
    assert "extinguish" in sent[-1]["objective"]
    ack(state, sent)
    demo.tick()
    assert demo.phase == "SECURING" and len(sent) == 1
    fire_robot(state, "MISSION_COMPLETE", "B014 now advertises RESOLVED; returning to the entrance")
    demo.tick()
    assert len(sent) == 1  # fire out, but the fire robot is still in the tunnels
    fire_robot(state, "MISSION_COMPLETE", "B014 now advertises RESOLVED; back at the entrance")
    demo.tick()
    assert (sent[-1]["robot_id"], sent[-1]["target_beacon_ids"]) == ("executor", [11, 21])
    ack(state, sent)
    demo.tick()
    assert demo.phase == "EXECUTING"
    assert any("Gas: no robot" in event["message"] for event in demo.events)


def fire_robot(state, value, detail=""):
    snapshot = state.snapshot()
    snapshot["robots"]["firebot"] = {"state": value, "detail": detail}
    state.replace(snapshot)


def _fire_out_and_returning():
    state, sent, now, demo = fleet_demo()
    discoveries(state)
    set_writer(state, "FAILED")
    demo.request("dispatch")
    ack(state, sent)
    demo.tick()
    fire_robot(state, "MISSION_COMPLETE", "returning to the entrance")
    demo.tick()
    return state, sent, now, demo


def test_rescue_robot_waits_while_the_fire_robot_is_still_driving_home():
    # A loaded machine runs the simulation slower than real time: minutes of wall clock
    # must not send the rescue robot into tunnels the fire robot is still driving through.
    state, sent, now, demo = _fire_out_and_returning()
    now[0] += 300
    demo.tick()
    assert sent[-1]["robot_id"] == "firebot" and demo.phase == "SECURING"


def test_rescue_robot_is_sent_when_the_fire_robot_cannot_get_home():
    state, sent, now, demo = _fire_out_and_returning()
    fire_robot(state, "MISSION_COMPLETE", "could not get back to the entrance")
    demo.tick()
    assert sent[-1]["robot_id"] == "executor"


def test_rescue_robot_is_sent_after_the_safety_timeout():
    state, sent, now, demo = _fire_out_and_returning()
    now[0] += 601
    demo.tick()
    assert sent[-1]["robot_id"] == "executor"


def test_mission_completes_only_when_every_dispatched_robot_is_done():
    state, sent, now, demo = fleet_demo()
    discoveries(state)
    set_writer(state, "FAILED")
    demo.request("dispatch")
    ack(state, sent)
    demo.tick()
    fire_robot(state, "MISSION_COMPLETE", "back at the entrance")
    demo.tick()
    ack(state, sent)
    demo.tick()
    fire_robot(state, "MISSION_RUNNING")
    snapshot = state.snapshot()
    snapshot["robots"]["executor"]["state"] = "MISSION_COMPLETE"
    state.replace(snapshot)
    demo.tick()
    assert demo.phase == "EXECUTING"
    fire_robot(state, "MISSION_COMPLETE", "back at the entrance")
    demo.tick()
    assert demo.phase == "COMPLETE" and "fire robot put the fire out" in demo.message


def test_resolved_targets_are_never_dispatched_again():
    from living_map_command_post.demo import plan_dispatch
    snapshot = {"robots": {"executor": {"state": "READY"}, "firebot": {"state": "READY"}},
                "memories": [{"mission_id": 1, "beacon_id": 3, "event_type": 4, "freshness": "FRESH",
                              "roles": ["STANDOFF", "RESOLVED"]},
                             {"mission_id": 1, "beacon_id": 2, "event_type": 1, "freshness": "FRESH"}]}
    assert [(robot, [t["beacon_id"] for t in targets]) for robot, targets in plan_dispatch(snapshot)] == \
        [("executor", [2])]


def test_victims_are_visited_by_priority_not_by_distance():
    from living_map_command_post.demo import plan_dispatch
    snapshot = {"robots": {"executor": {"state": "READY"}},
                "memories": [
                    {"mission_id": 1, "beacon_id": 2, "event_type": 1, "severity": 3, "confidence": 0.96,
                     "freshness": "FRESH"},
                    {"mission_id": 1, "beacon_id": 5, "event_type": 1, "severity": 4, "confidence": 0.90,
                     "freshness": "FRESH"},
                    {"mission_id": 1, "beacon_id": 7, "event_type": 1, "severity": 4, "confidence": 0.95,
                     "freshness": "AGING"}]}
    (robot, targets), = plan_dispatch(snapshot)
    assert robot == "executor" and [t["beacon_id"] for t in targets] == [7, 5, 2]


def test_scripted_demo_waits_for_both_victims():
    state, sent, now, demo = setup_demo()
    demo.request("full_demo")
    ack(state, sent)
    demo.tick()
    snapshot = state.snapshot()
    snapshot["memories"] = [{"mission_id": 1, "beacon_id": 10 + k, "event_type": k, "freshness": "FRESH",
                             "severity": 3, "confidence": 0.9} for k in (1, 2, 4)]
    state.replace(snapshot)
    demo.tick()
    assert len(sent) == 1  # one victim so far: keep exploring
    discoveries(state)
    demo.tick()
    assert sent[-1]["action"] == "fail_writer"


def test_writer_destroyed_out_of_radio_reach_still_hands_over():
    # Destroyed deep in the mine, its last FAILED transmission never got out: the command
    # post only sees it fall silent, still reporting its last state.
    state, sent, now, demo = fleet_demo()
    discoveries(state)
    demo.request("fail_writer")
    ack(state, sent)
    demo.tick()
    assert demo.phase == "WAITING_FAILURE"
    snapshot = state.snapshot()
    snapshot["robots"]["writer"]["comms"] = "SILENT"
    state.replace(snapshot)
    demo.tick()
    assert demo.phase == "MEMORY_PRESERVED"
    demo.request("dispatch")
    assert sent[-1]["action"] == "dispatch"


def test_silent_exploring_writer_is_not_treated_as_lost():
    # Behind a rock wall an exploring Writer can be silent for a while: no hand-over.
    state, sent, now, demo = fleet_demo()
    discoveries(state)
    snapshot = state.snapshot()
    snapshot["robots"]["writer"]["comms"] = "SILENT"
    state.replace(snapshot)
    with pytest.raises(Exception):
        demo.request("dispatch")
