from __future__ import annotations

import copy
import threading
import time
from collections.abc import Callable


class GatewayState:
    """Thread-safe state shared between ROS callbacks and the HTTP server."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.memories: dict[tuple[int, int], dict] = {}
        self.robots: dict[str, dict] = {}
        self.mission: dict = {"status": "IDLE"}
        self.missions: dict[str, dict] = {}
        self.network: dict = {"gateway": "ONLINE", "packets_received": 0, "packets_dropped": 0}
        self.started = time.time()
        self.demo = {}
        self.live_map = None
        self.mesh: dict = {}
        self.hops: dict[int, int] = {}
        self.destroyed: set[int] = set()

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "memories": copy.deepcopy(list(self.memories.values())),
                "robots": copy.deepcopy(self.robots),
                "mission": copy.deepcopy(self.mission),
                "missions": copy.deepcopy(self.missions),
                "network": copy.deepcopy(self.network),
                "elapsed_s": time.time() - self.started,
                "generated_at": time.time(),
                "demo": copy.deepcopy(self.demo),
                "map": copy.deepcopy(self.live_map),
                "mesh": copy.deepcopy(self.mesh),
            }

    def update_demo(self, **fields):
        with self.lock:
            self.demo.update(copy.deepcopy(fields))

    def update_map(self, data):
        with self.lock:
            self.live_map = data

    def update_mesh(self, topology: dict) -> None:
        """Keep a compact routing tree for the bandwidth-limited uplink.

        The full link list grows quadratically with beacons; the command post
        only needs each beacon's next hop toward the gateway.
        """
        routes = {}
        for beacon_id, route in (topology.get("routes") or {}).items():
            path = route.get("path") or []
            routes[beacon_id] = {
                "next": path[1] if len(path) > 1 else "gateway",
                "hops": route.get("hops"),
                "p": route.get("p"),
            }
        with self.lock:
            self.mesh = {
                "beacons": copy.deepcopy(topology.get("beacons") or {}),
                "routes": routes,
                "dead": list(topology.get("dead") or []),
                "unreachable": list(topology.get("unreachable") or []),
                "robots": copy.deepcopy(topology.get("robots") or {}),
                "link_count": len(topology.get("links") or []),
            }
            for robot_id, route in (topology.get("robots") or {}).items():
                if robot_id in self.robots:
                    self.robots[robot_id]["route"] = copy.deepcopy(route)

    def record_hops(self, beacon_id: int, hops: int) -> None:
        with self.lock:
            self.hops[beacon_id] = hops
            for (_mission_id, stored_id), memory in self.memories.items():
                if stored_id == beacon_id:
                    memory["hops"] = hops

    def mark_beacon_destroyed(self, beacon_id: int) -> None:
        with self.lock:
            self.destroyed.add(beacon_id)
            for (_mission_id, stored_id), memory in self.memories.items():
                if stored_id == beacon_id:
                    memory["destroyed"] = True

    def memory(self, mission_id: int, beacon_id: int) -> dict | None:
        with self.lock:
            memory = self.memories.get((mission_id, beacon_id))
            return copy.deepcopy(memory) if memory is not None else None

    def mark_silent_robots(self, now: float, silent_after_s: float) -> None:
        """Telemetry that stops arriving means the robot left radio reach or died."""
        with self.lock:
            for robot in self.robots.values():
                last_seen = robot.get("last_seen")
                if last_seen is not None:
                    robot["comms"] = "LIVE" if now - float(last_seen) <= silent_after_s else "SILENT"

    def record_packet_drop(self) -> None:
        with self.lock:
            self.network["packets_dropped"] = int(self.network.get("packets_dropped", 0)) + 1

    def record_packet_received(self) -> None:
        with self.lock:
            self.network["packets_received"] = int(self.network.get("packets_received", 0)) + 1

    def upsert_memory(self, mission_id: int, beacon_id: int, memory: dict) -> None:
        with self.lock:
            previous = self.memories.get((mission_id, beacon_id), {})
            if previous.get("executor_confirmed"):
                memory["executor_confirmed"] = True
                memory["confirmed_at"] = previous.get("confirmed_at")
            if beacon_id in self.hops:
                memory.setdefault("hops", self.hops[beacon_id])
            if beacon_id in self.destroyed:
                memory["destroyed"] = True
            self.memories[(mission_id, beacon_id)] = memory

    def confirm_beacon(self, beacon_id: int, confirmed_at: float) -> None:
        with self.lock:
            for (_mission_id, stored_id), memory in self.memories.items():
                if stored_id == beacon_id:
                    memory["executor_confirmed"] = True
                    memory["confirmed_at"] = float(confirmed_at)

    def update_robot(self, robot_id: str, data: dict) -> None:
        with self.lock:
            self.robots[robot_id] = data
            mission = self.missions.get(robot_id)
            if mission is None:
                return
            status = {"MISSION_COMPLETE": "MISSION COMPLETE", "MISSION_FAILED": "MISSION FAILED",
                      "NAVIGATING": "EN ROUTE", "ASSISTING": "ASSISTING VICTIM",
                      "EXTINGUISHING": "EXTINGUISHING FIRE", "REPORTING": "UPDATING BEACON"}.get(data.get("state"))
            if status and mission.get("status") not in {"MISSION COMPLETE", "MISSION FAILED"}:
                mission["status"] = status
            if robot_id == self.mission.get("robot_id"):
                self.mission = copy.deepcopy(mission)

    def robot(self, robot_id: str) -> dict:
        with self.lock:
            return copy.deepcopy(self.robots.get(robot_id, {}))

    def update_network(self, **fields) -> None:
        with self.lock:
            self.network.update(fields)

    def set_mission(self, robot_id: str, mission: dict) -> None:
        """Record a robot's mission; ``mission`` mirrors the most recently changed one."""
        with self.lock:
            self.missions[robot_id] = copy.deepcopy(mission)
            self.mission = copy.deepcopy(mission)

    def age_memories(self, scale: float, classify: Callable[[float, int, int], str]) -> None:
        now = time.time()
        with self.lock:
            for memory in self.memories.values():
                age_s = max(0.0, now - float(memory["written_at"])) * scale
                memory["age_s"] = round(age_s, 1)
                memory["freshness"] = classify(
                    age_s, int(memory["ttl_sec"]), int(memory.get("event_type", 0))
                )
