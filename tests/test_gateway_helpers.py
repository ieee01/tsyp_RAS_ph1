import json
import time

import pytest

from living_map_gateway.coordinate import map_bearing_to_compass
from living_map_gateway.map_codec import coarse_runs, known_fraction
from living_map_gateway.state import GatewayState


def test_map_bearing_becomes_a_compass_bearing():
    assert map_bearing_to_compass(0.0, 0.0) == (90.0, "E")       # map +X points East
    assert map_bearing_to_compass(0.0, 90.0) == (0.0, "N")
    degrees, point = map_bearing_to_compass(0.0, 18.0)          # demo anchor heading
    assert degrees == pytest.approx(72.0) and point == "ENE"


def test_coarse_map_keeps_walls_and_unknown():
    data = [0, 0, 100, -1,
            0, 0, -1, -1]
    width, height, runs = coarse_runs(data, 4, 2, step=2)
    assert (width, height) == (2, 1)
    assert runs == [[0, 1], [100, 1]]
    assert known_fraction(data) == pytest.approx(5 / 8)


def test_compact_mesh_and_silent_robots():
    state = GatewayState()
    state.update_robot("writer", {"state": "ONLINE", "last_seen": time.time() - 10})
    state.update_mesh({"beacons": {"1": [0, 0], "2": [5, 0]},
                       "links": [{"a": 1, "b": 2}] * 50,
                       "routes": {"1": {"path": [1], "hops": 1, "p": 0.98},
                                  "2": {"path": [2, 1], "hops": 2, "p": 0.9}},
                       "robots": {"writer": {"connected": False}}})
    state.mark_silent_robots(time.time(), 4.0)
    snapshot = state.snapshot()
    assert snapshot["mesh"]["routes"]["2"]["next"] == 1
    assert snapshot["mesh"]["routes"]["1"]["next"] == "gateway"
    assert snapshot["mesh"]["link_count"] == 50 and "links" not in snapshot["mesh"]
    assert snapshot["robots"]["writer"]["comms"] == "SILENT"
    assert snapshot["robots"]["writer"]["route"] == {"connected": False}


def test_hops_and_destroyed_flags_survive_memory_refresh():
    state = GatewayState()
    memory = {"beacon_id": 4, "written_at": 0, "ttl_sec": 1800, "event_type": 0}
    state.upsert_memory(1, 4, dict(memory))
    state.record_hops(4, 3)
    state.mark_beacon_destroyed(4)
    state.upsert_memory(1, 4, dict(memory))
    stored = state.memory(1, 4)
    assert stored["hops"] == 3 and stored["destroyed"] is True


def test_snapshot_fits_the_long_distance_link_budget():
    """A large mission still fits well inside one second of the 32 KB/s uplink."""
    state = GatewayState()
    for beacon_id in range(1, 31):
        state.upsert_memory(1, beacon_id, {
            "beacon_id": beacon_id, "mission_id": 1, "sequence": beacon_id, "roles": ["BREADCRUMB"],
            "x": round(beacon_id * 0.7, 2), "y": -2.0, "event_type": 0,
            "severity": 0, "previous_beacon_id": beacon_id - 1, "next_beacon_id": beacon_id + 1,
            "guidance": {"bearing_deg": 0.0, "compass_deg": 72.0, "compass": "ENE", "range_m": 6.0, "to": "B002"},
            "confidence": 1.0, "written_at": 0, "ttl_sec": 1800, "age_s": 10.0, "freshness": "FRESH",
            "gps": {"lat": 36.8065123, "lon": 10.1815123}, "hops": 3})
    state.update_mesh({"beacons": {str(b): [b, 0] for b in range(1, 31)},
                       "links": [{"a": a, "b": b} for a in range(30) for b in range(a)],
                       "routes": {str(b): {"path": list(range(b, 0, -1)), "hops": b, "p": 0.9} for b in range(1, 31)}})
    state.update_map({"width": 160, "height": 60, "runs": [[0, 40], [100, 3], [-1, 20]] * 120})
    for robot in ("writer", "executor"):
        state.update_robot(robot, {"state": "ONLINE", "history": [[round(i * 0.13, 2), -2.25] for i in range(120)]})
    assert len(json.dumps(state.snapshot(), separators=(",", ":"))) < 20000


def test_command_post_keeps_the_map_when_the_gateway_omits_it():
    from living_map_command_post.state import CommandPostState
    state = CommandPostState()
    state.replace({"map": {"width": 2}, "map_version": 5.0})
    state.replace({"map_version": 5.0, "memories": []})
    assert state.snapshot()["map"] == {"width": 2}
    state.replace({"map_version": 6.0})
    assert state.snapshot().get("map") is None
