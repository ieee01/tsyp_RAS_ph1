from __future__ import annotations

import json
import math
import queue
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ByteMultiArray, Float32MultiArray, String, UInt16
from std_srvs.srv import SetBool, Trigger

from living_map_beacons.aging import freshness
from living_map_beacons.protocol import FLAG_STANDOFF, FRAME_SIZE, decode, roles, target_position
from living_map_interfaces.msg import BeaconSeedBatch, MissionBrief, RobotStatus

from .coordinate import map_bearing_to_compass, map_to_wgs84
from .dedup import DedupCache
from .forward import ForwardDecision, LatestSnapshotForwarder
from .map_codec import coarse_runs, known_fraction
from .state import GatewayState

EVENT_NAMES = {1: "VICTIM", 2: "GAS_HAZARD", 3: "BLOCKED_PATH", 4: "FIRE"}
# Telemetry older than this means the robot is out of radio reach (or gone).
SILENT_AFTER_S = 4.0


def _frame_bytes(data) -> bytes:
    try:
        return bytes(data)
    except TypeError:
        return b"".join(value if isinstance(value, bytes) else bytes((value,)) for value in data)


class Gateway(Node):
    """The Outside Network Area: the only bridge between the mine radio and the world.

    Inside, it hears the beacon mesh and robot telemetry relayed over it.
    Outside, it talks to the distant command post over the long-distance link.
    It validates (CRC, dedup), translates map coordinates to WGS84, keeps the
    latest knowledge through link outages, and routes briefs to the right robot.
    """

    def __init__(self) -> None:
        super().__init__("gateway", namespace="/living_map")
        self.declare_parameter("gps_anchor", "config/gps_anchor.yaml")
        self.declare_parameter("age_scale", 1.0)
        # Rescue robots the Outside Network can brief and route messages to.
        self.declare_parameter("fleet", ["executor"])
        self.fleet = [str(robot) for robot in self.get_parameter("fleet").value]

        self.state = GatewayState()
        self.cache = DedupCache()
        self.anchor = self._anchor()
        # Keep the latest frame per (mission, beacon). Mission IDs are part of the
        # key so reused beacon IDs cannot overwrite another mission's memory.
        self.raw_frames: dict[tuple[int, int], tuple[int, bytes]] = {}
        self.mission_requests: queue.SimpleQueue[dict] = queue.SimpleQueue()
        self.forwarder = LatestSnapshotForwarder()
        self.last_snapshot = ""
        self.control_acks = {}
        self.pending_controls = set()
        self.full_map: OccupancyGrid | None = None
        self.map_received_at: float | None = None
        self.topology: dict = {}
        self.sent_map_version: float | None = None
        self.sent_map_at = 0.0
        self.fail_client = self.create_client(Trigger, "writer/fail")
        self.enable_client = self.create_client(SetBool, "writer/set_enabled")
        self.stuck_client = self.create_client(SetBool, "writer/set_stuck")

        self.mission_pub = self.create_publisher(MissionBrief, "inside/mission_brief", 10)
        # Data routing: each robot has its own seed channel.
        self.seed_pubs = {robot: self.create_publisher(BeaconSeedBatch, f"inside/{robot}_seed_batch", 10)
                          for robot in self.fleet}
        self.link_pub = self.create_publisher(String, "link/gateway_tx", 50)
        self.disable_beacon_pub = self.create_publisher(UInt16, "radio/disable_beacon", 10)
        self.create_subscription(ByteMultiArray, "gateway/rf_rx", self.on_packet, 50)
        self.create_subscription(RobotStatus, "gateway/robot_status", self.on_robot, 20)
        self.create_subscription(Float32MultiArray, "radio/metrics", self.on_radio_metric, 50)
        self.create_subscription(String, "radio/topology", self.on_topology, 5)
        self.create_subscription(Bool, "link/status", self.on_link_status, 10)
        self.create_subscription(String, "gateway/link_rx", self.on_link_command, 20)
        self.create_subscription(UInt16, "gateway/beacon_confirmed", self.on_beacon_confirmed, 20)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "gateway/map_rx", self.on_map, map_qos)
        self.create_timer(1.0, self.age_tick)
        self.create_timer(0.1, self.process_mission_requests)
        self.create_timer(1.0, self.publish_snapshot)

    # -- inbound: map, frames, telemetry -----------------------------------
    def on_map(self, msg: OccupancyGrid) -> None:
        """Keep the full-resolution map for briefings; send a coarse copy outside."""
        self.full_map = msg
        self.map_received_at = time.time()
        width, height, runs = coarse_runs(msg.data, msg.info.width, msg.info.height)
        self.state.update_map({"width": width, "height": height,
                               "resolution": msg.info.resolution * 4,
                               "origin_x": msg.info.origin.position.x,
                               "origin_y": msg.info.origin.position.y,
                               "frame": msg.header.frame_id, "runs": runs,
                               "known_fraction": round(known_fraction(msg.data), 3),
                               "received_at": self.map_received_at})

    def _anchor(self) -> dict:
        anchor_path = Path(str(self.get_parameter("gps_anchor").value))
        try:
            return yaml.safe_load(anchor_path.read_text())
        except Exception as exc:
            self.get_logger().error(f"GPS anchor load failed ({anchor_path}): {exc}; using safe demo defaults")
            return {
                "latitude": 36.8065,
                "longitude": 10.1815,
                "altitude": 10.0,
                "heading_deg": 0.0,
                "map_x": -1.5,
                "map_y": 0.0,
            }

    def _gps(self, x: float, y: float) -> dict:
        lat, lon = map_to_wgs84(
            x,
            y,
            self.anchor["latitude"],
            self.anchor["longitude"],
            self.anchor.get("heading_deg", 0.0),
            self.anchor.get("map_x", 0.0),
            self.anchor.get("map_y", 0.0),
        )
        return {"lat": round(lat, 7), "lon": round(lon, 7)}

    def on_packet(self, msg: ByteMultiArray) -> None:
        frame = _frame_bytes(msg.data)
        try:
            packet = decode(frame)
        except Exception as exc:
            self.state.record_packet_drop()
            self.get_logger().warning(f"Invalid beacon packet: {exc}")
            return

        frame_key = (packet.mission_id, packet.beacon_id)
        previous = self.raw_frames.get(frame_key)
        if previous is None or packet.sequence >= previous[0]:
            self.raw_frames[frame_key] = (packet.sequence, frame)

        dedup_key = (packet.mission_id, packet.beacon_id, packet.sequence)
        if not self.cache.accept(dedup_key):
            return
        if previous is not None and packet.sequence < previous[0]:
            return  # An older copy relayed late must not roll a memory back.

        target_x, target_y = target_position(packet)
        compass, point = map_bearing_to_compass(packet.bearing_rad, self.anchor.get("heading_deg", 0.0))
        guidance = None
        if packet.range_m > 0:
            to = (EVENT_NAMES.get(packet.event_type, "EVENT").replace("_", " ").lower()
                  if packet.flags & FLAG_STANDOFF else
                  (f"B{packet.next_beacon_id:03d}" if packet.next_beacon_id else "next beacon"))
            guidance = {"bearing_deg": round(math.degrees(packet.bearing_rad), 1),
                        "compass_deg": round(compass, 1), "compass": point,
                        "range_m": round(packet.range_m, 2), "to": to}
        self.state.record_packet_received()
        memory = {
            "beacon_id": packet.beacon_id,
            "mission_id": packet.mission_id,
            "sequence": packet.sequence,
            "roles": roles(packet.flags),
            "x": round(packet.x_m, 2),
            "y": round(packet.y_m, 2),
            "event_type": packet.event_type,
            "severity": packet.severity,
            "previous_beacon_id": packet.previous_beacon_id,
            "next_beacon_id": packet.next_beacon_id,
            "guidance": guidance,
            "confidence": packet.confidence,
            "written_at": packet.timestamp_sec,
            "ttl_sec": packet.ttl_sec,
            "age_s": round(max(0.0, time.time() - packet.timestamp_sec), 1),
            "freshness": "FRESH",
            # GPS of the WHAT: the event itself for standoff beacons.
            "gps": self._gps(target_x, target_y),
        }
        if (target_x, target_y) != (packet.x_m, packet.y_m):
            memory.update(target_x=round(target_x, 2), target_y=round(target_y, 2),
                          beacon_gps=self._gps(packet.x_m, packet.y_m))
        self.state.upsert_memory(packet.mission_id, packet.beacon_id, memory)
        self.get_logger().info(
            f"B{packet.beacon_id:03d} seq={packet.sequence} CRC valid "
            f"[{'/'.join(roles(packet.flags)) or 'NAV'}] local=({packet.x_m:.2f},{packet.y_m:.2f})"
        )

    def on_robot(self, msg: RobotStatus) -> None:
        previous = self.state.robot(msg.robot_id)
        history = previous.get("history", [])
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not history or (history[-1][0] - x) ** 2 + (history[-1][1] - y) ** 2 > 0.1**2:
            history = (history + [[round(x, 2), round(y, 2)]])[-120:]
        self.state.update_robot(
            msg.robot_id,
            {
                "state": msg.state,
                "x": round(x, 3),
                "y": round(y, 3),
                "frame": msg.pose.header.frame_id or "map",
                "detail": msg.detail,
                "last_seen": time.time(),
                "comms": "LIVE",
                "battery": round(float(msg.battery_percent), 1) if msg.battery_percent >= 0 else None,
                "history": history,
                "gps": self._gps(x, y),
                "route": (self.topology.get("robots") or {}).get(msg.robot_id),
            },
        )

    def on_radio_metric(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            return
        to_gateway = msg.data[5] < 0.5
        self.state.update_network(
            last_rssi_dbm=float(msg.data[3]),
            last_delivery_probability=float(msg.data[2]),
            last_destination="gateway" if to_gateway else "executor",
        )
        if to_gateway and len(msg.data) >= 7 and msg.data[4] >= 0.5:
            self.state.record_hops(int(msg.data[0]), int(msg.data[6]))
        if to_gateway and msg.data[4] < 0.5:
            self.state.record_packet_drop()

    def on_topology(self, msg: String) -> None:
        try:
            self.topology = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.state.update_mesh(self.topology)

    def on_beacon_confirmed(self, msg: UInt16) -> None:
        self.state.confirm_beacon(int(msg.data), time.time())

    def age_tick(self) -> None:
        scale = float(self.get_parameter("age_scale").value)
        self.state.age_memories(
            scale, lambda age, ttl, event_type: freshness(
                age, ttl, event_type=event_type
            ).value
        )
        self.state.mark_silent_robots(time.time(), SILENT_AFTER_S)

    # -- outbound: long-distance link -------------------------------------
    def on_link_status(self, msg: Bool) -> None:
        decision = self.forwarder.set_link(bool(msg.data))
        self.state.update_network(
            long_distance_link="ONLINE" if self.forwarder.link_up else "OUTAGE",
            queued_snapshots=self.forwarder.queued_snapshots,
        )
        if decision is not None:
            self._send_snapshot(decision)
            self.get_logger().info(
                "Long-distance link restored; sent latest snapshot and discarded "
                f"{decision.discarded_snapshots} superseded snapshots"
            )

    def publish_snapshot(self) -> None:
        state = self.state.snapshot()
        # The map is the largest item: send it only when it changed, plus a
        # periodic refresh. The command post keeps its last copy in between.
        version = (state.get("map") or {}).get("received_at")
        now = time.time()
        if version is not None and version == self.sent_map_version and now - self.sent_map_at < 10.0:
            state.pop("map", None)
        else:
            self.sent_map_version, self.sent_map_at = version, now
        state["map_version"] = version
        snapshot = json.dumps(state, separators=(",", ":"), sort_keys=True)
        if snapshot == self.last_snapshot and self.forwarder.queued_snapshots == 0:
            return
        self.last_snapshot = snapshot
        decision = self.forwarder.offer(snapshot)
        self.state.update_network(queued_snapshots=self.forwarder.queued_snapshots)
        if decision is not None:
            self._send_snapshot(decision)

    def _send_snapshot(self, decision: ForwardDecision) -> None:
        """Publish one selected snapshot and expose recovery discard metrics."""
        msg = String()
        msg.data = decision.payload
        self.link_pub.publish(msg)
        self.state.update_network(
            queued_snapshots=self.forwarder.queued_snapshots,
            stale_snapshots_discarded=decision.discarded_snapshots,
        )

    # -- commands from the command post ------------------------------------
    def on_link_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejected malformed command-post message: {exc}")
            return
        if isinstance(payload, dict) and payload.get("kind") == "demo_control":
            self.on_demo_command(payload)
            return
        if not isinstance(payload, dict):
            self.get_logger().warning("Rejected invalid command-post mission")
            return
        error = self.request_mission(payload)
        if error:
            self.get_logger().warning(f"Rejected command-post mission: {error}")

    def acknowledge(self, request_id, success, message):
        ack = {"request_id": request_id, "success": bool(success), "message": message}
        self.pending_controls.discard(request_id)
        self.control_acks[request_id] = ack
        if len(self.control_acks) > 128:
            self.control_acks.pop(next(iter(self.control_acks)))
        self.state.update_demo(ack=ack)

    def _robot_route(self, robot_id: str) -> dict:
        return (self.topology.get("robots") or {}).get(robot_id) or {}

    def _robot_reachable(self, robot_id: str) -> bool:
        return bool(self._robot_route(robot_id).get("connected"))

    def _call(self, request_id: str, client, request) -> None:
        if not client.service_is_ready():
            self.acknowledge(request_id, False, "Writer control service is not ready")
            return
        self.pending_controls.add(request_id)
        future = client.call_async(request)

        def done(result):
            try:
                response = result.result()
                self.acknowledge(request_id, response.success, response.message)
            except Exception as exc:
                self.acknowledge(request_id, False, str(exc))
        future.add_done_callback(done)

    def on_demo_command(self, payload):
        request_id = payload.get("request_id")
        action = payload.get("action")
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
            return
        if request_id in self.control_acks:
            self.state.update_demo(ack=self.control_acks[request_id])
            return
        if request_id in self.pending_controls:
            return
        if action == "dispatch":
            error = self.request_mission(payload)
            if error:
                self.acknowledge(request_id, False, error)
                return
            self.pending_controls.add(request_id)
            return
        if action in {"start", "pause"}:
            # A real radio command: it only reaches a Writer the mesh can route to.
            if not self._robot_reachable("writer"):
                self.acknowledge(request_id, False, "Writer is out of radio reach; command cannot be delivered")
                return
            self._call(request_id, self.enable_client, SetBool.Request(data=action == "start"))
            return
        # Fault injection: simulation-harness actions, not radio commands.
        if action == "fail_writer":
            self._call(request_id, self.fail_client, Trigger.Request())
            return
        if action == "stuck_writer":
            self._call(request_id, self.stuck_client, SetBool.Request(data=True))
            return
        if action == "destroy_beacon":
            try:
                beacon_id = int(payload.get("beacon_id"))
            except (TypeError, ValueError):
                beacon_id = 0
            if not any(key[1] == beacon_id for key in self.raw_frames):
                self.acknowledge(request_id, False, f"Unknown beacon {beacon_id}")
                return
            self.disable_beacon_pub.publish(UInt16(data=beacon_id))
            self.state.mark_beacon_destroyed(beacon_id)
            self.acknowledge(request_id, True, f"B{beacon_id:03d} destroyed (fault injection); mesh re-routing")
            return
        self.acknowledge(request_id, False, "Unsupported demo action")

    def request_mission(self, payload: dict) -> str | None:
        """Validate WHO and WHAT; return an error message, or None when queued.

        The command post names a robot and one or more target memories, in the
        order it wants them handled. How to get there is the robot's decision,
        so no route or hazard policy is accepted here.
        """
        try:
            mission_id = int(payload.get("mission_id", 1))
            targets = [int(t) for t in (payload.get("target_beacon_ids") or [payload.get("target_beacon_id", 0)])]
        except (TypeError, ValueError):
            return "Invalid mission identifiers"
        robot_id = str(payload.get("robot_id", "executor"))
        if not 1 <= mission_id <= 65535:
            return "Invalid mission ID"
        if robot_id not in self.fleet:
            return f"Unknown robot '{robot_id}'"
        if not targets or len(targets) > 16 or len(set(targets)) != len(targets):
            return "A mission needs 1 to 16 distinct targets"
        event_types = set()
        for target in targets:
            memory = self.state.memory(mission_id, target)
            if memory is None:
                return f"No memory B{target:03d} in mission {mission_id}"
            if int(memory.get("event_type", 0)) not in EVENT_NAMES:
                return "Targets must be event memories, not navigation beacons"
            if memory.get("freshness") == "EXPIRED":
                return f"Memory B{target:03d} has expired"
            event_types.add(int(memory["event_type"]))
        if len(event_types) != 1:
            return "All targets of one mission must be the same kind of event"
        robot = self.state.robot(robot_id)
        if robot.get("state") not in {"READY", "MISSION_FAILED"}:
            return f"{robot_id.capitalize()} is not ready for a new mission"
        route = self._robot_route(robot_id)
        if not route.get("connected") or route.get("hops") != 1:
            return f"{robot_id.capitalize()} must be at the entrance, in direct range of the gateway, to be briefed"

        request = dict(payload, mission_id=mission_id, robot_id=robot_id, target_beacon_ids=targets,
                       target_beacon_id=targets[0], target_type=event_types.pop())
        self.mission_requests.put(request)
        self.state.set_mission(robot_id, {"status": "REQUESTED", "mission_id": mission_id, "robot_id": robot_id,
                                          "target_beacon_id": targets[0]})
        return None

    def process_mission_requests(self) -> None:
        while True:
            try:
                payload = self.mission_requests.get_nowait()
            except queue.Empty:
                return
            self._publish_mission(payload)
            if payload.get("kind") == "demo_control":
                self.acknowledge(payload["request_id"], True, "Mission brief, memory seed and map delivered to Executor")

    def _publish_mission(self, payload: dict) -> None:
        target_type = int(payload["target_type"])
        targets = [int(t) for t in payload["target_beacon_ids"]]
        target_beacon_id = targets[0]
        label = " → ".join(f"{EVENT_NAMES.get(target_type, 'EVENT')} @ B{t:03d}" for t in targets)
        mission = MissionBrief()
        mission.mission_id = int(payload["mission_id"])
        mission.robot_id = str(payload["robot_id"])
        mission.target_beacon_id = target_beacon_id
        mission.target_beacon_ids = targets
        mission.target_event = label
        mission.target_type = target_type
        mission.avoid_event_types = []
        mission.objective = str(payload.get("objective") or f"Reach {label} using inherited memories.")[:256]
        mission.issued_at = self.get_clock().now().to_msg()

        # Seed the Executor transactionally in one ROS message: every frame the
        # outside world holds plus the last map the Writer uploaded. The Executor
        # only plans once it has this exact batch, so nothing can race ahead.
        frames = [
            frame
            for (mission_id, _beacon_id), (_sequence, frame) in sorted(self.raw_frames.items())
            if mission_id == mission.mission_id
        ]
        batch = BeaconSeedBatch()
        batch.mission_id = mission.mission_id
        batch.issued_at = mission.issued_at
        batch.frame_size = FRAME_SIZE
        batch.frame_count = len(frames)
        batch.data = b"".join(frames)
        if self.full_map is not None:
            batch.map = self.full_map
        self.seed_pubs[mission.robot_id].publish(batch)
        self.mission_pub.publish(mission)

        map_age = None if self.map_received_at is None else round(time.time() - self.map_received_at, 1)
        self.state.set_mission(
            mission.robot_id,
            {
                "status": "BRIEFED",
                "robot_id": mission.robot_id,
                "target_event": label,
                "target_beacon_id": target_beacon_id,
                "target_beacon_ids": targets,
                "target_type": target_type,
                "mission_id": mission.mission_id,
                "seeded_memories": len(frames),
                "map_included": self.full_map is not None,
                "map_age_s": map_age,
                "objective": mission.objective,
            }
        )
        self.get_logger().info(
            f"Mission {mission.mission_id} briefed to {mission.robot_id} through the gateway: target {label}; "
            f"seeded {len(frames)} memories, map {'included' if self.full_map is not None else 'unavailable'}"
        )


def main() -> None:
    rclpy.init()
    node = Gateway()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
