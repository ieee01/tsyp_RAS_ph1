"""Browser demo sequence driven solely by gateway acknowledgements and telemetry."""
from __future__ import annotations

import copy
import threading
import time
import uuid


class DemoError(ValueError):
    pass


# The automatic demo waits for VICTIM, GAS_HAZARD and FIRE before ending the
# Writer's sortie. A manual brief only needs a victim memory; the Executor
# avoids every hazard it inherits by its own policy.
REQUIRED_EVENT_TYPES = {1, 2, 4}
# Demo script: the default scenario has two victims; the automatic sequence lets
# the Writer find both before its sortie ends, so the priority order is visible.
DEMO_MIN_VICTIMS = 2
TARGET_EVENT_TYPE = 1
EVENT_NAMES = {1: "VICTIM", 2: "GAS_HAZARD", 3: "BLOCKED_PATH", 4: "FIRE"}

# How the Writer's sortie ends in the automatic demo.
OUTCOMES = {
    "destroyed": "Writer destroyed after recording the events",
    "stuck": "Writer immobilised after recording the events; it keeps relaying",
    "return": "Best case: Writer maps the mine and drives back to the entrance",
}
# Writer states in which it can no longer carry the mission itself.
HANDOVER_STATES = {"FAILED", "STUCK", "HOME"}
# Longest wait for the fire robot to clear the tunnels before the rescue robot goes in.
SECURE_TIMEOUT_S = 600

# The fleet the command post can send. It decides WHO goes and WHAT the target
# is; each robot decides HOW to get there.
FLEET = {
    "firebot": {"model": "fire robot", "capabilities": {4}, "role": "extinguish fires"},
    "executor": {"model": "rescue robot", "capabilities": {1}, "role": "reach and assist victims"},
}
# Which event types the command post sends a robot to, in dispatch order: clear
# fires first, then reach victims. Gas has no capable robot and is only avoided.
MISSION_TYPES = [4, 1]
READY_STATES = {"READY", "MISSION_FAILED"}
FRESHNESS_RANK = {"FRESH": 0, "AGING": 1, "STALE": 2, "EXPIRED": 3}


def live_memories(snapshot, mission_id=1):
    return [m for m in snapshot.get("memories", [])
            if int(m.get("mission_id", 0)) == mission_id and m.get("freshness") != "EXPIRED"]


def is_resolved(memory):
    return "RESOLVED" in (memory.get("roles") or [])


def priority(memory):
    """Most urgent first: severity, then confidence, then freshness, then newest."""
    return (
        -int(memory.get("severity", 0)),
        -float(memory.get("confidence", 0.0)),
        FRESHNESS_RANK.get(memory.get("freshness"), 3),
        float(memory.get("age_s", 0.0)),
        int(memory.get("beacon_id", 0)),
    )


def select_targets(snapshot, event_type=TARGET_EVENT_TYPE, mission_id=1):
    """WHAT, in order: every live, unresolved memory of a type, by priority."""
    candidates = [m for m in live_memories(snapshot, mission_id)
                  if int(m.get("event_type", 0)) == event_type and not is_resolved(m)]
    return sorted(candidates, key=priority)


def select_target(snapshot, event_type=TARGET_EVENT_TYPE, mission_id=1):
    """WHAT: the most urgent, most trustworthy, freshest memory of a type."""
    targets = select_targets(snapshot, event_type, mission_id)
    return targets[0] if targets else None


def select_robot(snapshot, event_type=TARGET_EVENT_TYPE, exclude=()):
    """WHO: a ready robot whose capabilities match the objective."""
    robots = snapshot.get("robots", {})
    for robot_id, spec in FLEET.items():
        if robot_id in exclude or event_type not in spec["capabilities"]:
            continue
        if robots.get(robot_id, {}).get("state") in READY_STATES:
            return robot_id
    return None


def plan_dispatch(snapshot):
    """WHO goes WHERE: one capable ready robot per event type, its targets in priority order."""
    plan, used = [], set()
    for event_type in MISSION_TYPES:
        targets = select_targets(snapshot, event_type)
        robot_id = select_robot(snapshot, event_type, exclude=used) if targets else None
        if targets and robot_id:
            plan.append((robot_id, targets))
            used.add(robot_id)
    return plan


class DemoController:
    def __init__(self, state, send, mode="autonomous", clock=time.monotonic, speed="normal",
                 min_gas_hazards=1):
        self.state = state
        self.send = send
        self.clock = clock
        self.lock = threading.RLock()
        self.mode = mode
        self.speed = speed
        self.min_gas_hazards = min_gas_hazards
        self.outcome = "destroyed"
        self.phase = "READY"
        self.message = "Start exploration or run the full sequence."
        self.automatic = False
        self.pending = None
        self.deadline = 0.0
        self.events = []
        self.returning_logged = False
        self.dispatch_queue = []
        self.dispatched = []
        self.securing_since = None
        # Set once the Writer was destroyed and went radio silent before its last FAILED
        # transmission could get out: it is lost, even though its last report says otherwise.
        self.writer_lost = False
        self._log("Command post ready")

    def _log(self, message):
        self.events.append({"time": time.strftime("%H:%M:%S"), "message": message})
        self.events = self.events[-40:]

    def note(self, message):
        """Record a harness action on the run timeline."""
        with self.lock:
            self._log(message)

    def snapshot(self):
        with self.lock:
            return {"phase": self.phase, "message": self.message,
                    "automatic": self.automatic, "busy": self.pending is not None,
                    "navigation_mode": self.mode, "speed": self.speed, "outcome": self.outcome,
                    "gas_hazards_required": self.min_gas_hazards,
                    "fleet": {robot_id: {"model": spec["model"], "role": spec["role"]}
                              for robot_id, spec in FLEET.items()},
                    "events": copy.deepcopy(self.events)}

    @staticmethod
    def live_event_types(snapshot):
        return {int(m.get("event_type", 0)) for m in live_memories(snapshot)}

    @classmethod
    def has_events(cls, snapshot, min_gas_hazards=1):
        memories = live_memories(snapshot)
        victims = sum(1 for m in memories if int(m.get("event_type", 0)) == 1)
        gas_hazards = sum(1 for m in memories if int(m.get("event_type", 0)) == 2)
        return (REQUIRED_EVENT_TYPES <= cls.live_event_types(snapshot)
                and victims >= DEMO_MIN_VICTIMS and gas_hazards >= min_gas_hazards)

    @classmethod
    def has_target(cls, snapshot):
        return TARGET_EVENT_TYPE in cls.live_event_types(snapshot)

    def _transition(self, phase, message, timeout=300):
        self.phase, self.message = phase, message
        self.deadline = self.clock() + timeout
        self._log(message)

    def _command(self, action, **fields):
        request = {"kind": "demo_control", "action": action, "request_id": uuid.uuid4().hex, **fields}
        self.pending = {"payload": request, "attempts": 1, "sent_at": self.clock()}
        self.send(request)
        self._log(f"Sent {action.replace('_', ' ')} through the gateway uplink")

    def _dispatch(self, snapshot):
        if not self.has_target(snapshot):
            raise DemoError("Record a victim memory before dispatching.")
        plan = plan_dispatch(snapshot)
        if not any(robot_id == "executor" for robot_id, _ in plan):
            raise DemoError("No ready robot can take the victim mission. Choose New demo for a replay.")
        for robot_id, targets in plan:
            what = " then ".join(
                f"{EVENT_NAMES[int(t['event_type'])]} B{int(t['beacon_id']):03d} (severity {t.get('severity', '?')}, "
                f"{round(float(t.get('confidence', 0)) * 100)}%, {t.get('freshness', '?').lower()})" for t in targets)
            self._log(f"Decision: WHO = {robot_id} ({FLEET[robot_id]['model']}); WHAT = {what}. "
                      "HOW is left to the robot.")
        if any(int(m.get("event_type", 0)) == 2 for m in live_memories(snapshot)):
            self._log("Gas: no robot can neutralise it; every robot keeps it out of its route.")
        self.dispatched = [robot_id for robot_id, _ in plan]
        self.dispatch_queue = list(plan)
        self._send_next_brief()

    def _send_next_brief(self):
        robot_id, targets = self.dispatch_queue.pop(0)
        beacons = [int(t["beacon_id"]) for t in targets]
        fire = int(targets[0]["event_type"]) == 4
        action_text = "extinguish each fire" if fire else "assist each victim"
        names = ", ".join(f"B{b:03d}" for b in beacons)
        self._command("dispatch", mission_id=1, robot_id=robot_id, target_beacon_id=beacons[0],
                      target_beacon_ids=beacons,
                      objective=f"In priority order ({names}), reach and {action_text} using inherited memories.")

    def request(self, action, outcome=None, beacon_id=None):
        with self.lock:
            snapshot = self.state.snapshot()
            writer = snapshot.get("robots", {}).get("writer", {}).get("state")
            executor = snapshot.get("robots", {}).get("executor", {}).get("state")
            if action == "cancel_auto":
                self.automatic = False
                self.message = "Automatic sequencing cancelled. Use the manual controls."
                self._log(self.message)
                return
            if self.pending:
                raise DemoError("Waiting for the gateway to acknowledge the previous command.")
            # Destroying a beacon is a fault a judge may inject at any time, even mid-sequence.
            if self.automatic and action != "destroy_beacon":
                raise DemoError("A sequence is running. Wait for acknowledgement or cancel automatic sequencing.")
            if (snapshot.get("received_age_s") if snapshot.get("received_age_s") is not None else 99) > 5 or writer is None:
                raise DemoError("Waiting for fresh gateway telemetry. Commands are unavailable while the link is stale.")
            if action in {"start", "full_demo", "pause"} and writer == "FAILED":
                raise DemoError("Writer has failed. Choose New demo to restart.")
            if action in {"start", "full_demo"} and writer in {"STUCK", "HOME"}:
                raise DemoError(f"Writer is {writer.lower()}. Brief the Executor or choose New demo.")
            if action == "fail_writer" and writer == "FAILED":
                raise DemoError("Writer is already failed. Choose New demo for a replay.")
            if action == "stuck_writer" and writer in {"FAILED", "STUCK"}:
                raise DemoError(f"Writer is already {writer.lower()}.")
            if action == "dispatch":
                if writer not in HANDOVER_STATES and not self.writer_lost:
                    raise DemoError("The Writer is still on its sortie. Brief the Executor once it is "
                                    "destroyed, stuck or back home.")
                if not self.has_target(snapshot):
                    raise DemoError("Record the victim memory before dispatching.")
                if executor not in READY_STATES:
                    raise DemoError("The rescue robot already has a mission. Choose New demo for a replay.")
            if action == "full_demo":
                if executor != "READY":
                    raise DemoError("Choose New demo before starting another sequence.")
                if outcome is not None:
                    if outcome not in OUTCOMES:
                        raise DemoError("Unknown Writer outcome")
                    self.outcome = outcome
                self.automatic = True
                self._transition("STARTING", f"Starting Writer exploration · scenario: {OUTCOMES[self.outcome]}")
                self._command("start")
            elif action == "dispatch":
                self._dispatch(snapshot)
            elif action == "destroy_beacon":
                if beacon_id is None:
                    raise DemoError("Choose a beacon to destroy.")
                self._command("destroy_beacon", beacon_id=int(beacon_id))
            elif action in {"start", "pause", "fail_writer", "stuck_writer"}:
                self._command(action)
            else:
                raise DemoError("Unknown demo action")

    def _auto_dispatch(self, snapshot):
        try:
            self._dispatch(snapshot)
        except DemoError as exc:
            self._fail(str(exc))

    def _after_events(self):
        """End the Writer's sortie the way the chosen scenario says."""
        if self.outcome == "destroyed":
            self._transition("FAILING_WRITER", "Both victims, gas hazards and fire recorded; injecting Writer destruction", 30)
            self._command("fail_writer")
        elif self.outcome == "stuck":
            self._transition("STICKING_WRITER", "Both victims, gas hazards and fire recorded; immobilising the Writer", 30)
            self._command("stuck_writer")

    def tick(self):
        with self.lock:
            snapshot = self.state.snapshot()
            if self.pending:
                ack = snapshot.get("demo", {}).get("ack", {})
                payload = self.pending["payload"]
                if ack.get("request_id") == payload["request_id"]:
                    self.pending = None
                    if not ack.get("success"):
                        self._fail(ack.get("message", "Gateway rejected the command"))
                        return
                    self._log(ack.get("message", "Gateway command acknowledged"))
                    action = payload["action"]
                    if action == "start":
                        waiting = ("Writer exploring until the mine is mapped, then returning home"
                                   if self.automatic and self.outcome == "return"
                                   else "Writer exploring; waiting for both victims, all gas hazards and fire")
                        # Exploration time varies with the order frontiers are visited.
                        self._transition("EXPLORING", waiting, 900 if self.outcome == "return" else 600)
                    elif action == "pause":
                        self._transition("PAUSED", "Writer paused; memories remain available")
                    elif action == "fail_writer":
                        self._transition("WAITING_FAILURE", "Waiting for the Writer's last FAILED transmission", 30)
                    elif action == "stuck_writer":
                        self._transition("WAITING_STUCK", "Waiting for STUCK telemetry relayed over the mesh", 30)
                    elif action == "dispatch":
                        if self.dispatch_queue:
                            # Secure the scene first: the next robot goes in once the
                            # fire robot is done and back out of the tunnels.
                            self._transition("SECURING", "Fire robot briefed; the rescue robot goes in once the fire "
                                                         "is out and the fire robot is back at the entrance", 600)
                            self.securing_since = None
                        else:
                            robots = " and ".join(FLEET[r]["model"] for r in self.dispatched)
                            self._transition("EXECUTING", f"Briefed the {robots}; each navigates with its own logic "
                                                          "from inherited memories", 900)
                    elif action == "destroy_beacon":
                        self._log(f"Beacon B{int(payload.get('beacon_id', 0)):03d} destroyed; the mesh re-routes around it")
                elif self.clock() - self.pending["sent_at"] > 8:
                    if self.pending["attempts"] >= 5:
                        self.pending = None
                        self._fail("No gateway acknowledgement. Check the link, then retry.")
                    else:
                        self.pending["attempts"] += 1
                        self.pending["sent_at"] = self.clock()
                        self.send(payload)  # Same ID: gateway deduplicates retries.
                return
            if (snapshot.get("received_age_s") if snapshot.get("received_age_s") is not None else 99) > 5:
                if self.automatic and self.clock() > self.deadline:
                    self._fail("Gateway telemetry became stale. Restore the link and retry.")
                return
            writer = snapshot.get("robots", {}).get("writer", {}).get("state")
            silent = snapshot.get("robots", {}).get("writer", {}).get("comms") == "SILENT"
            if self.phase == "WAITING_FAILURE" and (writer == "FAILED" or silent):
                if writer != "FAILED":
                    # Destroyed out of radio reach: the dying gasp never got out.
                    self.writer_lost = True
                    self._log("Writer radio silent after destruction; treated as lost")
                self._transition("MEMORY_PRESERVED", "Writer destroyed; persistent memories survived")
                if self.automatic:
                    self._auto_dispatch(snapshot)
            elif self.phase == "WAITING_STUCK" and writer == "STUCK":
                self._transition("MEMORY_PRESERVED", "Writer stuck but alive; memories and relay survive")
                if self.automatic:
                    self._auto_dispatch(snapshot)
            elif self.phase in {"EXPLORING", "PAUSED"} and writer == "STUCK":
                self._transition("MEMORY_PRESERVED", "Writer reported itself STUCK; memories and relay survive")
                if self.automatic and self.has_target(snapshot):
                    self._auto_dispatch(snapshot)
            elif self.phase == "EXPLORING" and writer == "RETURNING" and not self.returning_logged:
                self.returning_logged = True
                self._log("Writer finished exploring and is returning to the entrance")
            elif self.phase == "EXPLORING" and writer == "HOME":
                self.returning_logged = False
                self._transition("MEMORY_PRESERVED", "Best case: Writer is home with the full map")
                if self.automatic:
                    if self.has_target(snapshot):
                        self._auto_dispatch(snapshot)
                    else:
                        self._fail("Writer mapped the mine but recorded no victim; nothing to dispatch.")
            elif (self.phase == "EXPLORING" and self.automatic and self.outcome != "return"
                  and self.has_events(snapshot, self.min_gas_hazards)):
                self._after_events()
            elif self.phase == "SECURING":
                fire = snapshot.get("robots", {}).get("firebot", {})
                if fire.get("state") == "MISSION_FAILED":
                    self._fail(f"The fire robot stopped its mission: {fire.get('detail', '')}")
                elif fire.get("state") == "MISSION_COMPLETE":
                    if self.securing_since is None:
                        self.securing_since = self.clock()
                        self._log("Fire out and beacon updated; fire robot returning to the entrance")
                    home = "back at the entrance" in fire.get("detail", "") or "could not get back" in fire.get("detail", "")
                    # Wait for the fire robot's own report (home, or could not get back). The
                    # timeout is only a safety net: it counts wall-clock seconds, and a loaded
                    # machine runs the simulation several times slower than real time.
                    if home or self.clock() - self.securing_since > SECURE_TIMEOUT_S:
                        self._log("Scene secured: briefing the rescue robot")
                        self._send_next_brief()
            elif self.phase == "EXECUTING" and self.dispatched and all(
                    snapshot.get("robots", {}).get(r, {}).get("state") == "MISSION_COMPLETE" for r in self.dispatched):
                self.automatic = False
                done = []
                if "firebot" in self.dispatched:
                    done.append("the fire robot put the fire out")
                detail = snapshot.get("robots", {}).get("executor", {}).get("detail", "")
                victims = len((snapshot.get("missions", {}).get("executor") or {}).get("target_beacon_ids") or [1])
                assisted = max(1, detail.count("now advertises RESOLVED"))
                missed = detail.count("not reached")
                assisted = min(assisted, victims - missed) if missed else assisted
                if missed:
                    done.append(f"the rescue robot assisted {assisted} of {victims} victims; {missed} could not be reached")
                else:
                    done.append("the rescue robot reached and assisted " +
                                (f"{victims} victims in priority order" if victims > 1 else "the victim"))
                self._transition("COMPLETE", "Mission complete: " + " and ".join(done) +
                                 "; each marked its beacon resolved. Extraction is outside Phase 1.")
            elif self.phase == "EXECUTING" and any(
                    snapshot.get("robots", {}).get(r, {}).get("state") == "MISSION_FAILED" for r in self.dispatched):
                robot_id = next(r for r in self.dispatched
                                if snapshot.get("robots", {}).get(r, {}).get("state") == "MISSION_FAILED")
                detail = snapshot.get("robots", {}).get(robot_id, {}).get("detail", "")
                self._fail(f"The {FLEET[robot_id]['model']} stopped its mission: {detail}" if detail else
                           f"The {FLEET[robot_id]['model']} stopped its mission. Start a new demo.")
            elif self.automatic and self.clock() > self.deadline:
                self._fail("Sequence timed out. Inspect exploration or replay in the labelled guided demo mode.")

    def _fail(self, message):
        self.automatic = False
        self._transition("ERROR", message)
