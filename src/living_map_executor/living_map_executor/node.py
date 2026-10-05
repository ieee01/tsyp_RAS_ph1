from __future__ import annotations

import math
import copy
import json
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ByteMultiArray, String, UInt16
from tf2_ros import Buffer, TransformException, TransformListener

from living_map_beacons.aging import freshness
from living_map_beacons.protocol import (FLAG_RESOLVED, FRAME_SIZE, BeaconPacket, decode, encode, resolve_update,
                                         target_position)
from living_map_interfaces.msg import BeaconSeedBatch, MissionBrief, RobotStatus
from living_map_writer.battery import Battery

from .graph import MemoryGraph
from .policy import avoid_types, edge_cost, freshness_cost, keepout_radius, stamp_keepout
from .route import (clear_spawn_footprint, footprint_is_free, hazard_avoiding_waypoints, nearest_clear_waypoint,
                    segment_distance, segment_is_clear, shortcut_waypoints, approach_from_first_in_reach)

EVENT_NAMES = {1: "VICTIM", 2: "GAS_HAZARD", 3: "BLOCKED_PATH", 4: "FIRE"}


def _byte_array(data) -> bytes:
    """Normalize rclpy uint8[] values across rmw implementations."""
    try:
        return bytes(data)
    except TypeError:
        return b"".join(
            value if isinstance(value, bytes) else bytes((value,))
            for value in data
        )


class Executor(Node):
    """Execute an inherited mission from persistent beacon memories.

    All the Executor knows comes from the Outside Network brief (beacon frames
    plus the last map the Writer uploaded) and from beacons it hears itself.
    It plans in the shared ``map`` frame and cannot plan a newly briefed
    mission until the matching transactional seed batch has been imported.
    """

    def __init__(self) -> None:
        super().__init__("executor", namespace="/living_map")
        # One node per rescue robot; every topic and frame derives from its ID.
        self.declare_parameter("robot_id", "executor")
        self.declare_parameter("max_goal_retries", 3)
        self.declare_parameter("retry_delay_sec", 1.5)
        # The enlarged Husky needs room to stop outside the mapped victim visual.
        self.declare_parameter("target_tolerance_m", 1.7)
        self.declare_parameter("waypoint_clearance_m", 1.1)
        # Where the robot stops to act, leave room to turn around and leave again.
        self.declare_parameter("final_clearance_m", 1.4)
        self.declare_parameter("hazard_block_radius_m", 2.5)
        self.declare_parameter("graph_connect_radius_m", 4.5)
        self.declare_parameter("target_protect_radius_m", 1.0)
        # Fire is approached from outside its flames before extinguishing it.
        self.declare_parameter("fire_tolerance_m", 2.8)
        self.declare_parameter("fire_core_m", 1.0)
        self.declare_parameter("assist_s", 3.0)
        self.declare_parameter("extinguish_s", 5.0)
        self.declare_parameter("write_back_timeout_s", 20.0)
        self.declare_parameter("battery_start_pct", 100.0)
        # After its last objective the robot can drive back to where it started,
        # clearing the tunnels for other robots (used by the fire robot).
        self.declare_parameter("return_home_after_mission", False)

        self.robot_id = str(self.get_parameter("robot_id").value)
        rid = self.robot_id
        self.battery = Battery(level_pct=float(self.get_parameter("battery_start_pct").value))
        self.last_tick_s: Optional[float] = None
        self.home: Optional[tuple[float, float]] = None
        self.home_goal: Optional[tuple[float, float]] = None  # home, or the nearest mapped free spot
        self.homing = ""
        self.home_handle = None
        self.declare_parameter("home_tolerance_m", 0.6)
        # Intermediate beacons are pass-through points: within this distance, go on to the
        # next one instead of fine-positioning a long robot on the exact spot.
        self.declare_parameter("pass_waypoint_m", 1.0)
        self.passing_waypoint = False
        self.mission: Optional[MissionBrief] = None
        self.pose: Optional[tuple[float, float]] = None
        self.memories: dict[int, BeaconPacket] = {}
        self.graph = MemoryGraph()
        self.route: list[tuple[float, float]] = []
        self.route_labels: list[str] = []
        self.target: Optional[BeaconPacket] = None
        self.target_point: Optional[tuple[float, float]] = None
        self.inherited_map: Optional[OccupancyGrid] = None
        self.base_map: Optional[OccupancyGrid] = None
        self.map: Optional[OccupancyGrid] = None
        self.stamped_hazards: frozenset[int] = frozenset()

        self.seed_ready_mission_id: Optional[int] = None
        self.seed_ready_token: Optional[tuple[int, int]] = None
        self.active = False
        self.complete = False
        self.failed = False
        self.failure_reason = ""
        self.completion_note = ""
        self.current_goal: Optional[tuple[float, float]] = None
        self.goal_handle = None
        self.replan_requested = False
        self.skipped_waypoints = 0
        self._edge_ok = None
        self.goal_retries = 0
        self.next_retry_at = 0.0

        # Telemetry goes to the Executor's radio; the mesh decides if it gets out.
        self.status_pub = self.create_publisher(RobotStatus, f"inside/{rid}_status", 10)
        self.confirmed_pub = self.create_publisher(UInt16, f"inside/{rid}_beacon_confirmed", 20)
        self.write_pub = self.create_publisher(ByteMultiArray, f"inside/{rid}_beacon_write", 10)
        self.extinguish_pub = self.create_publisher(String, "sim/extinguish", 10)
        self.navigation_map_pub = self.create_publisher(
            OccupancyGrid, f"/{rid}/navigation_map",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.nav = ActionClient(self, NavigateToPose, f"/{rid}/navigate_to_pose")
        # Mission action and write-back state.
        self.phase = ""
        self.targets: list[int] = []
        self.target_index = 0
        self.handled: list[str] = []
        self.unreached: list[str] = []
        self.action_until = 0.0
        self.pending_write: Optional[BeaconPacket] = None
        self.write_deadline = 0.0
        self.write_confirmed = False
        self.last_write_sent = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(MissionBrief, "inside/mission_brief", self.on_brief, 10)
        self.create_subscription(BeaconSeedBatch, f"inside/{rid}_seed_batch", self.on_seed_batch, 10)
        self.create_subscription(ByteMultiArray, f"{rid}/rf_rx", self.on_live_packet, 20)
        self.create_timer(0.5, self.tick)

    # -- inherited map ----------------------------------------------------
    def _prepare_inherited_map(self) -> bool:
        """Clear the parked Executor's own imprint from the briefed map."""
        if self.inherited_map is None:
            return False
        message = self.inherited_map
        try:
            transform = self.tf_buffer.lookup_transform("map", f"{self.robot_id}/odom", Time())
        except TransformException:
            return False
        base = copy.deepcopy(message)
        base.data = clear_spawn_footprint(
            message.data, message.info.width, message.info.height, message.info.resolution,
            (message.info.origin.position.x, message.info.origin.position.y),
            (transform.transform.translation.x, transform.transform.translation.y),
            float(self.get_parameter("waypoint_clearance_m").value),
        )
        self.base_map = base
        self.inherited_map = None
        self.stamped_hazards = frozenset()
        self._restamp_hazards(force=True)
        return True

    def _hazards(self) -> list[tuple[int, BeaconPacket, tuple[float, float]]]:
        if self.mission is None:
            return []
        avoid = avoid_types(self.mission.avoid_event_types)
        return [
            (beacon_id, packet, target_position(packet))
            for beacon_id, packet in self.memories.items()
            if packet.mission_id == self.mission.mission_id and packet.event_type in avoid
            and beacon_id != self._current_target_id()
            and not packet.flags & FLAG_RESOLVED  # an extinguished fire no longer blocks the way
        ]

    def _restamp_hazards(self, force: bool = False) -> bool:
        """Write every known hazard into the Nav2 planning map as a hard keep-out.

        Returns True when the set of stamped hazards changed.
        """
        if self.base_map is None:
            return False
        hazards = self._hazards()
        ids = frozenset(beacon_id for beacon_id, _, _ in hazards)
        if not force and ids == self.stamped_hazards:
            return False
        info = self.base_map.info
        protect = []
        discs = [(x, y, keepout_radius(packet.event_type)) for _, packet, (x, y) in hazards]
        if self.target_point is not None and self.target is not None:
            if self.target.event_type == 4 and not self.target.flags & FLAG_RESOLVED:
                # Even the fire we are sent to fight keeps a lethal core.
                discs.append((*self.target_point, float(self.get_parameter("fire_core_m").value)))
            else:
                protect.append((*self.target_point, float(self.get_parameter("target_protect_radius_m").value)))
        planning = copy.deepcopy(self.base_map)
        planning.data = stamp_keepout(
            self.base_map.data, info.width, info.height, info.resolution,
            (info.origin.position.x, info.origin.position.y),
            discs,
            protect,
        )
        planning.header.stamp = self.get_clock().now().to_msg()
        self.map = planning
        self.navigation_map_pub.publish(planning)
        changed = ids != self.stamped_hazards
        self.stamped_hazards = ids
        if hazards:
            self.get_logger().info(
                "Hazard keep-out stamped into planning map: "
                + ", ".join(f"B{bid:03d} {EVENT_NAMES.get(p.event_type, '?')} r={keepout_radius(p.event_type):.1f} m"
                            for bid, p, _ in hazards)
            )
        return changed

    def is_map_free(self, x: float, y: float) -> bool:
        """Return true only for a known, low-cost occupancy-grid cell."""
        if self.map is None or self.map.info.resolution <= 0.0:
            return False
        column = math.floor((x - self.map.info.origin.position.x) / self.map.info.resolution)
        row = math.floor((y - self.map.info.origin.position.y) / self.map.info.resolution)
        if column < 0 or row < 0:
            return False
        if column >= self.map.info.width or row >= self.map.info.height:
            return False
        value = int(self.map.data[row * self.map.info.width + column])
        return 0 <= value <= 20

    def is_map_occupied(self, x: float, y: float) -> bool:
        """Return true only for a known occupied cell; unknown space is not a wall."""
        if self.map is None or self.map.info.resolution <= 0.0:
            return False
        column = math.floor((x - self.map.info.origin.position.x) / self.map.info.resolution)
        row = math.floor((y - self.map.info.origin.position.y) / self.map.info.resolution)
        if not (0 <= column < self.map.info.width and 0 <= row < self.map.info.height):
            return False
        return int(self.map.data[row * self.map.info.width + column]) >= 65

    def _segment_clear_of_walls(self, a: tuple[float, float], b: tuple[float, float]) -> bool:
        if self.map is None:
            return True
        return segment_is_clear(a, b, self.is_map_occupied, self.map.info.resolution * 0.5, end_margin=0.6)

    def update_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform("map", f"{self.robot_id}/base_link", rclpy.time.Time())
            self.pose = (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except TransformException:
            # Map->executor TF is expected shortly after SLAM and Gazebo startup.
            return

    # -- memory intake ----------------------------------------------------
    def on_seed_batch(self, msg: BeaconSeedBatch) -> None:
        if msg.frame_size != FRAME_SIZE:
            self.get_logger().error(
                f"Rejected mission seed: frame_size={msg.frame_size}, expected {FRAME_SIZE}"
            )
            return

        raw = _byte_array(msg.data)
        expected_size = int(msg.frame_count) * int(msg.frame_size)
        if len(raw) != expected_size:
            self.get_logger().error(
                f"Rejected mission seed: got {len(raw)} bytes, expected {expected_size}"
            )
            return

        # A seed batch is a mission-consistent snapshot. Rebuild local memory from
        # it rather than mixing stale memories from an earlier mission.
        self.memories.clear()
        for offset in range(0, len(raw), FRAME_SIZE):
            frame = raw[offset : offset + FRAME_SIZE]
            if not self._import_frame(frame, expected_mission_id=int(msg.mission_id)):
                self.get_logger().warning("Mission seed contained an invalid or expired frame")

        self.base_map = None
        self.map = None
        self.inherited_map = msg.map if msg.map.info.width > 0 and msg.map.info.resolution > 0 else None
        if self.inherited_map is None:
            self.get_logger().warning("Brief carried no map: planning from beacon memories and live LiDAR only")
        self.seed_ready_mission_id = int(msg.mission_id)
        self.seed_ready_token = (int(msg.issued_at.sec), int(msg.issued_at.nanosec))
        self.get_logger().info(
            f"Mission {msg.mission_id} seed complete: imported {len(self.memories)} memories"
            f"{' and the Writer map' if self.inherited_map is not None else ''} from the gateway"
        )
        self._prepare_inherited_map()
        self.maybe_plan()

    def on_live_packet(self, msg: ByteMultiArray) -> None:
        try:
            heard = decode(_byte_array(msg.data))
        except Exception as exc:
            self.get_logger().warning(f"Heard invalid beacon frame: {exc}")
            return
        age = max(0.0, time.time() - heard.timestamp_sec)
        if freshness(age, heard.ttl_sec, event_type=heard.event_type).value == "EXPIRED":
            return
        # Once a mission is briefed, only memories of that mission may enter
        # the graph; earlier missions' beacons can still be within RF range.
        expected_mission_id = int(self.mission.mission_id) if self.mission is not None else None
        if expected_mission_id is not None and heard.mission_id != expected_mission_id:
            return

        # Confirm every valid in-mission beacon heard, including re-hearings of
        # a memory already imported, so the Command Post sees physical contact.
        confirmation = UInt16()
        confirmation.data = heard.beacon_id
        self.confirmed_pub.publish(confirmation)

        if not self._import_frame(_byte_array(msg.data), expected_mission_id=expected_mission_id):
            return
        if self.mission is None or self.complete or self.failed:
            return
        if self.pending_write is not None and heard.beacon_id == self.pending_write.beacon_id \
                and heard.flags & FLAG_RESOLVED:
            self._write_confirmed()
            return
        if self.phase:
            return
        if self._restamp_hazards():
            # A new (or newly resolved) hazard changes the plan, even mid-route.
            self.get_logger().warning(f"Hazard picture changed by B{heard.beacon_id:03d}; replanning")
            self.route.clear()
            if self.active and self.goal_handle is not None:
                self.replan_requested = True
                self.goal_handle.cancel_goal_async()
                return
        self.maybe_plan()

    def _import_frame(self, frame: bytes, expected_mission_id: Optional[int] = None) -> bool:
        try:
            packet = decode(frame)
        except Exception as exc:
            self.get_logger().warning(f"Beacon rejected: {exc}")
            return False

        if expected_mission_id is not None and packet.mission_id != expected_mission_id:
            self.get_logger().warning(
                f"Beacon B{packet.beacon_id:03d} belongs to mission {packet.mission_id}, "
                f"not expected mission {expected_mission_id}"
            )
            return False

        age = max(0.0, time.time() - packet.timestamp_sec)
        if freshness(age, packet.ttl_sec, event_type=packet.event_type).value == "EXPIRED":
            return False

        previous = self.memories.get(packet.beacon_id)
        if previous is not None and previous.sequence >= packet.sequence:
            return False

        self.memories[packet.beacon_id] = packet
        self.get_logger().info(
            f"Imported memory B{packet.beacon_id:03d} mission={packet.mission_id} type={packet.event_type}"
        )
        return True

    def on_brief(self, msg: MissionBrief) -> None:
        if msg.robot_id and msg.robot_id != self.robot_id:
            return  # Routed to another robot of the fleet.
        self.mission = msg
        self.targets = [int(t) for t in msg.target_beacon_ids] or [int(msg.target_beacon_id)]
        self.target_index = 0
        self.handled = []
        self.unreached = []
        self.route.clear()
        self.route_labels = []
        self.target = None
        self.target_point = None
        self.active = False
        self.complete = False
        self.failed = False
        self.failure_reason = ""
        self.phase = ""
        self.skipped_waypoints = 0
        self.pending_write = None
        self.write_confirmed = False
        self.completion_note = ""
        self.current_goal = None
        self.goal_retries = 0
        self.next_retry_at = 0.0
        self.get_logger().info(
            f"Mission {msg.mission_id} inherited target={msg.target_event} (B{msg.target_beacon_id:03d}); "
            "waiting for matching memory seed if needed"
        )
        self.maybe_plan()

    def _seed_ready(self) -> bool:
        if self.mission is None:
            return False
        mission_token = (int(self.mission.issued_at.sec), int(self.mission.issued_at.nanosec))
        return (
            self.seed_ready_mission_id == int(self.mission.mission_id)
            and self.seed_ready_token == mission_token
        )

    def _tolerance(self) -> float:
        if self.target is not None and self.target.event_type == 4:
            return float(self.get_parameter("fire_tolerance_m").value)
        return float(self.get_parameter("target_tolerance_m").value)

    def _freshness(self, packet: BeaconPacket) -> str:
        age = max(0.0, time.time() - packet.timestamp_sec)
        return freshness(age, packet.ttl_sec, event_type=packet.event_type).value

    def _fail(self, reason: str) -> None:
        self.failed = True
        self.failure_reason = reason
        self.get_logger().error(f"{reason}; mission stopped")

    def _current_target_id(self) -> int:
        if self.targets and self.target_index < len(self.targets):
            return self.targets[self.target_index]
        return int(self.mission.target_beacon_id) if self.mission is not None else 0

    def _resolve_target(self) -> Optional[BeaconPacket]:
        """The current objective from the brief; legacy briefs fall back to the best of a type."""
        assert self.mission is not None
        if self._current_target_id():
            return self.memories.get(self._current_target_id())
        candidates = [p for p in self.memories.values()
                      if p.mission_id == self.mission.mission_id and p.event_type == self.mission.target_type]
        return max(candidates, key=lambda p: (p.severity, p.confidence, -freshness_cost(self._freshness(p))),
                   default=None)

    # -- planning ---------------------------------------------------------
    def _build_graph(self, blocked_node_ids: set[int], hazard_discs, target_id: int) -> MemoryGraph:
        graph = MemoryGraph()
        fresh = {}
        for beacon_id, packet in self.memories.items():
            graph.add(beacon_id, packet.x_m, packet.y_m, event_type=packet.event_type,
                      confidence=packet.confidence)
            fresh[beacon_id] = self._freshness(packet)

        def clear(a: int, b: int) -> bool:
            first, second = graph.nodes[a], graph.nodes[b]
            if not self._segment_clear_of_walls((first["x"], first["y"]), (second["x"], second["y"])):
                return False
            if target_id in (a, b):
                return True  # The final approach may be close to a hazard by design.
            return all(
                segment_distance(hx, hy, first["x"], first["y"], second["x"], second["y"]) > radius
                for hx, hy, radius in hazard_discs
            )

        # Links the Writer actually drove, in either direction of the chain.
        for beacon_id, packet in self.memories.items():
            for other in (packet.previous_beacon_id, packet.next_beacon_id):
                if other in self.memories and other != beacon_id and beacon_id not in blocked_node_ids \
                        and other not in blocked_node_ids and clear(beacon_id, other):
                    distance = math.dist((packet.x_m, packet.y_m),
                                         (self.memories[other].x_m, self.memories[other].y_m))
                    graph.connect(beacon_id, other, edge_cost(distance, True, fresh[beacon_id], fresh[other]))
        graph.connect_spatial_neighbors(
            float(self.get_parameter("graph_connect_radius_m").value),
            blocked_node_ids,
            edge_ok=clear,
            cost=lambda a, b, distance: edge_cost(distance, False, fresh[a], fresh[b]),
        )
        self._edge_ok = clear
        return graph

    def maybe_plan(self) -> None:
        if (
            self.mission is None
            or self.pose is None
            or not self._seed_ready()
            or self.active
            or self.complete
            or self.failed
            or self.phase
        ):
            return
        if self.inherited_map is not None and not self._prepare_inherited_map():
            return  # Waiting for the Executor's own frame to clear its spawn imprint.

        if self.route:
            self._next_goal()
            return

        target = self._resolve_target()
        if target is None:
            self._target_failed(f"B{self._current_target_id():03d} is not among the inherited memories")
            return
        if target.event_type == 0:
            self._target_failed(f"B{target.beacon_id:03d} is a navigation beacon, not an objective")
            return
        self.target = target
        self.target_point = target_position(target)
        self._restamp_hazards(force=True)

        hazard_radius = float(self.get_parameter("hazard_block_radius_m").value)
        hazards = self._hazards()
        hazard_points = [point for _, _, point in hazards]
        hazard_discs = [(x, y, hazard_radius) for x, y in hazard_points]
        probe = MemoryGraph()
        for beacon_id, packet in self.memories.items():
            probe.add(beacon_id, packet.x_m, packet.y_m)
        blocked_node_ids = probe.nodes_near_points(hazard_discs, keep={target.beacon_id})
        self.graph = self._build_graph(blocked_node_ids, hazard_discs, target.beacon_id)

        if blocked_node_ids:
            self.get_logger().warning(
                "Beacons inside hazard radius excluded: "
                + ", ".join(f"B{beacon_id:03d}" for beacon_id in sorted(blocked_node_ids))
            )

        usable = [beacon_id for beacon_id in self.graph.nodes if beacon_id not in blocked_node_ids]
        reachable = [beacon_id for beacon_id in usable
                     if self._segment_clear_of_walls(self.pose, (self.graph.nodes[beacon_id]["x"],
                                                                 self.graph.nodes[beacon_id]["y"]))]
        candidates = reachable or usable
        # Enter the beacon graph where the whole trip is shortest (distance to the
        # entry beacon + route from it to the target), not at the nearest beacon,
        # which may lie behind the robot.
        graph_path, best = [], math.inf
        for beacon_id in candidates:
            path = self.graph.shortest_path(beacon_id, target.beacon_id, (), blocked_node_ids, edge_ok=self._edge_ok)
            if not path:
                continue
            node = self.graph.nodes[beacon_id]
            cost = math.hypot(self.pose[0] - node["x"], self.pose[1] - node["y"]) + self.graph.path_cost(path)
            if cost < best:
                graph_path, best = path, cost
        target_label = f"{EVENT_NAMES.get(target.event_type, 'EVENT')} B{target.beacon_id:03d}"
        if graph_path:
            self.route = [
                (self.graph.nodes[beacon_id]["x"], self.graph.nodes[beacon_id]["y"])
                for beacon_id in graph_path
            ]
            self.route_labels = [f"B{beacon_id:03d}" for beacon_id in graph_path]
            if math.dist(self.route[-1], self.target_point) > 0.05:
                self.route.append(self.target_point)
                self.route_labels.append(target_label)
            if self.route and math.dist(self.route[0], self.pose) < 0.65:
                self.route.pop(0)
                self.route_labels.pop(0)
            before = len(self.route)
            self.route = shortcut_waypoints(
                self.pose, self.route, hazard_points, hazard_radius,
                is_clear=lambda a, b: self.map is not None and segment_is_clear(
                    a, b, self.is_map_occupied, self.map.info.resolution * 0.5),
            )
            self.route_labels = self.route_labels[before - len(self.route):]
            self.get_logger().info(
                "Inherited beacon route: "
                + " -> ".join(f"B{beacon_id:03d}" for beacon_id in graph_path)
                + f" -> {target_label}"
            )
        else:
            self.route = hazard_avoiding_waypoints(
                self.pose,
                self.target_point,
                hazard_points,
                1.6,
                is_free=self.is_map_free if self.map is not None else None,
                on_warning=self.get_logger().warning,
            )
            self.route_labels = ["detour"] * (len(self.route) - 1) + [target_label]
            self.get_logger().info(
                "Beacon graph incomplete; using semantic hazard-aware fallback route"
            )

        if target.event_type == 4:
            # A fire is fought from outside: approach it straight from its standoff
            # beacon and stop well inside the arrival distance (Nav2 may stop up to
            # 0.25 m short of a goal).
            self.route, self.route_labels = approach_from_first_in_reach(
                self.route, self.route_labels, self.target_point,
                reach=self._tolerance() + 0.6, stop=self._tolerance() - 0.35)

        if self.map is not None:
            safe_route, safe_labels = [], []
            for index, waypoint in enumerate(self.route):
                final = index == len(self.route) - 1
                max_offset = (self._tolerance() - 0.1
                              if final else 1.5)
                clearance = float(self.get_parameter("final_clearance_m" if final else "waypoint_clearance_m").value)
                # A wall-safe stop must still be close enough to perform the action.
                # Leave stopping margin for Nav2: fire uses a 0.25 m goal tolerance;
                # rescue uses 0.15 m so its shorter action range remains reachable.
                # Searching around the intended approach without this constraint can
                # move a fire stop several metres back down the corridor.
                def valid_stop(point, radius):
                    return (
                        (not final or math.dist(point, self.target_point)
                         <= self._tolerance() - (0.35 if target.event_type == 4 else 0.25))
                        and footprint_is_free(point, self.is_map_free, radius)
                        and (final or all(math.dist(point, hazard) > hazard_radius for hazard in hazard_points))
                    )

                safe = nearest_clear_waypoint(
                    waypoint,
                    lambda point: valid_stop(point, clearance),
                    max_offset, self.pose,
                )
                if safe is None and final:
                    # Fall back to the tighter clearance rather than give up on the target.
                    safe = nearest_clear_waypoint(
                        waypoint,
                        lambda point: valid_stop(point, float(self.get_parameter("waypoint_clearance_m").value)),
                        max_offset, self.pose,
                    )
                if safe is None:
                    if final:
                        self._target_failed(f"no footprint-safe approach to {target_label}")
                        return
                    self.get_logger().warning(f"Skipping unsafe beacon waypoint {waypoint}")
                    continue
                if math.dist(safe, waypoint) > 0.01:
                    self.get_logger().info(f"Moved wall-adjacent waypoint {waypoint} to footprint-safe {safe}")
                safe_route.append(safe)
                safe_labels.append(self.route_labels[index] if index < len(self.route_labels) else "")
            self.route, self.route_labels = safe_route, safe_labels

        self.get_logger().info(
            "New route: "
            + " -> ".join(f"({x:.1f},{y:.1f})" for x, y in self.route)
            + f" -> target {target_label}"
        )
        self._next_goal()

    # -- execution --------------------------------------------------------
    def _next_goal(self) -> None:
        if self.active or self.failed or self.complete or self.phase:
            return

        if not self.route:
            self._check_target_completion()
            return

        if time.monotonic() < self.next_retry_at:
            return

        if not self.nav.server_is_ready():
            self.get_logger().warning("Executor Nav2 action server is not ready; will retry")
            self.next_retry_at = time.monotonic() + float(self.get_parameter("retry_delay_sec").value)
            return

        self.current_goal = self.route[0]
        x, y = self.current_goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Face the direction of travel (toward the next waypoint, or toward the
        # target for the last one) so the robot drives through waypoints instead of
        # stopping to turn on the spot, which also needs room it may not have.
        if len(self.route) > 1:
            ahead = self.route[1]
        elif self.target_point is not None and math.dist(self.target_point, (x, y)) > 0.3:
            ahead = self.target_point
        else:
            ahead = None
        start = self.pose or (x - 1.0, y)
        yaw = math.atan2(ahead[1] - y, ahead[0] - x) if ahead else math.atan2(y - start[1], x - start[0])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.active = True
        self.nav.send_goal_async(goal).add_done_callback(self._goal_sent)

    def _goal_sent(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.active = False
            self._schedule_goal_retry(f"Executor Nav2 send failed: {exc}")
            return

        if not handle.accepted:
            self.active = False
            self._schedule_goal_retry("Executor Nav2 goal rejected")
            return

        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, future) -> None:
        self.active = False
        self.goal_handle = None
        if self.phase or self.complete:
            return  # Arrived by proximity (goal cancelled) or the mission is already over.
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self._schedule_goal_retry(f"Executor Nav2 result failed: {exc}")
            return

        if self.replan_requested:
            self.replan_requested = False
            self.passing_waypoint = False
            self.route.clear()
            self.current_goal = None
            self.maybe_plan()
            return

        if self.passing_waypoint:
            # Cancelled on purpose close to an intermediate beacon: treat it as reached.
            self.passing_waypoint = False
            status = GoalStatus.STATUS_SUCCEEDED

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.route:
                self.route.pop(0)
                if self.route_labels:
                    self.route_labels.pop(0)
            self.current_goal = None
            self.goal_retries = 0
            self.next_retry_at = 0.0
            if self.route:
                self._next_goal()
            else:
                self._check_target_completion()
            return

        if len(self.route) > 1 and self.skipped_waypoints < 4:
            # An intermediate waypoint is only a hint; skip it rather than fail the mission.
            self.skipped_waypoints += 1
            skipped = self.route.pop(0)
            if self.route_labels:
                self.route_labels.pop(0)
            self.get_logger().warning(f"Waypoint ({skipped[0]:.1f},{skipped[1]:.1f}) unreachable; skipping it")
            self.current_goal = None
            self._next_goal()
            return
        self._schedule_goal_retry(f"Executor navigation goal failed status={status}")

    def _schedule_goal_retry(self, reason: str) -> None:
        self.goal_retries += 1
        maximum = int(self.get_parameter("max_goal_retries").value)
        if self.goal_retries > maximum:
            self._target_failed(f"{reason}; retry limit exceeded")
            return

        delay = float(self.get_parameter("retry_delay_sec").value)
        self.next_retry_at = time.monotonic() + delay
        self.get_logger().warning(
            f"{reason}; retry {self.goal_retries}/{maximum} in {delay:.1f}s"
        )

    def _check_target_completion(self) -> None:
        self.update_pose()
        if self.target_point is None or self.pose is None:
            return

        distance = math.dist(self.pose, self.target_point)
        if distance <= self._tolerance():
            fire = self.target is not None and self.target.event_type == 4
            self.phase = "EXTINGUISHING" if fire else "ASSISTING"
            duration = float(self.get_parameter("extinguish_s" if fire else "assist_s").value)
            self.action_until = time.monotonic() + duration
            name = EVENT_NAMES.get(self.target.event_type, "target") if self.target is not None else "target"
            self.get_logger().info(
                f"Reached {name} B{self._current_target_id():03d} at {distance:.2f} m; "
                f"{self.phase.lower()} for {duration:.0f} s"
            )
            return

        # Never replace the validated approach with the occupied target center.
        self._target_failed(f"approach finished but the target is still {distance:.2f} m away")

    def _finish_action(self) -> None:
        """Action done: change the world if needed, then write the result into the beacon."""
        assert self.target is not None
        if self.target.event_type == 4 and self.target_point is not None:
            # Simulation harness: the fire goes out in the world.
            self.extinguish_pub.publish(String(data=json.dumps({
                "x": self.target_point[0], "y": self.target_point[1], "beacon_id": self.target.beacon_id})))
        latest = self.memories.get(self.target.beacon_id, self.target)
        self.pending_write = resolve_update(latest, int(time.time()))
        self.write_deadline = time.monotonic() + float(self.get_parameter("write_back_timeout_s").value)
        self.phase = "REPORTING"
        self.last_write_sent = 0.0
        self.get_logger().info(f"Writing RESOLVED into B{self.target.beacon_id:03d}")

    def _send_write(self) -> None:
        if self.pending_write is None:
            return
        msg = ByteMultiArray()
        msg.data = encode(self.pending_write)
        self.write_pub.publish(msg)
        self.last_write_sent = time.monotonic()

    def _write_confirmed(self) -> None:
        beacon_id = self.pending_write.beacon_id if self.pending_write else 0
        self.pending_write = None
        self.write_confirmed = True
        # Replace the original target packet with its acknowledged resolved version
        # and remove the fire core before planning the journey back out.
        self.target = self.memories.get(beacon_id, self.target)
        self._restamp_hazards(force=True)
        self._target_done(f"B{beacon_id:03d} now advertises RESOLVED")

    def _target_failed(self, reason: str) -> None:
        """One objective cannot be reached: report it and carry on with the others."""
        beacon_id = self._current_target_id()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.active = False
        self.unreached.append(f"B{beacon_id:03d} not reached ({reason})")
        self.get_logger().error(f"Objective B{beacon_id:03d} not reached: {reason}")
        if self.target_index + 1 < len(self.targets):
            self._next_target(f"B{beacon_id:03d} skipped")
        elif self.handled:
            self._complete("; ".join(self.handled + self.unreached))
        else:
            self._fail("; ".join(self.unreached))

    def _next_target(self, note: str) -> None:
        self.target_index += 1
        self.phase = ""
        self.route.clear()
        self.route_labels = []
        self.target = None
        self.target_point = None
        self.goal_retries = 0
        self.skipped_waypoints = 0
        self.get_logger().info(
            f"{note}; next objective {self.target_index + 1}/{len(self.targets)}: B{self._current_target_id():03d}")
        self.maybe_plan()

    def _target_done(self, note: str) -> None:
        """One objective handled: go on to the next by priority, or finish."""
        self.handled.append(note)
        if self.target_index + 1 < len(self.targets):
            self._next_target(note)
            return
        self._complete("; ".join(self.handled + self.unreached))

    def _complete(self, note: str) -> None:
        self.phase = ""
        self.complete = True
        self.active = False
        self.completion_note = note
        self.get_logger().info(f"MISSION COMPLETE: {note}")
        if bool(self.get_parameter("return_home_after_mission").value) and self.home is not None:
            self._drive_home()

    def _arrive_if_close(self) -> None:
        """Being within tolerance of the target is arrival, wherever the route stands."""
        if (self.mission is None or self.complete or self.failed or self.phase or self.target_point is None
                or self.pose is None or math.dist(self.pose, self.target_point) > self._tolerance()):
            return
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.route.clear()
        self.active = False
        self._check_target_completion()

    def _drive_home(self) -> None:
        if not self.nav.server_is_ready():
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        # The parking spot lies at the edge of the Writer's map and is often unknown
        # (or still shows this robot's own body), so Nav2 cannot plan into it. Park at
        # the nearest known-free spot with room to turn instead: still at the entrance.
        self.home_goal = self.home
        if self.map is not None:
            clearance = float(self.get_parameter("waypoint_clearance_m").value)
            self.home_goal = nearest_clear_waypoint(
                self.home, lambda point: footprint_is_free(point, self.is_map_free, clearance),
                3.0, self.pose or self.home) or self.home
        goal.pose.pose.position.x, goal.pose.pose.position.y = self.home_goal
        goal.pose.pose.orientation.w = 1.0
        self.homing = "returning to the entrance to clear the tunnels"
        self.get_logger().info("Mission done: returning to the entrance")

        def sent(future):
            try:
                handle = future.result()
            except Exception:
                return
            if handle.accepted:
                self.home_handle = handle
                handle.get_result_async().add_done_callback(done)

        def done(future):
            self.home_handle = None
            if self.homing == "back at the entrance":
                return  # Already stopped by proximity.
            try:
                ok = future.result().status == GoalStatus.STATUS_SUCCEEDED
            except Exception:
                ok = False
            self.homing = "back at the entrance" if ok else "could not get back to the entrance"
        self.nav.send_goal_async(goal).add_done_callback(sent)

    def _pass_waypoint_if_close(self) -> None:
        """Drive through intermediate beacons instead of stopping exactly on them."""
        if (not self.active or self.goal_handle is None or self.passing_waypoint or len(self.route) < 2
                or self.current_goal is None or self.pose is None
                or math.dist(self.pose, self.current_goal) > float(self.get_parameter("pass_waypoint_m").value)):
            return
        self.passing_waypoint = True
        self.get_logger().info(
            f"Passing waypoint ({self.current_goal[0]:.1f},{self.current_goal[1]:.1f}); on to the next")
        self.goal_handle.cancel_goal_async()

    def _stop_if_home(self) -> None:
        """Close enough to the parking spot is home: stop instead of fine-adjusting in place."""
        home = self.home_goal or self.home
        if (self.homing != "returning to the entrance to clear the tunnels" or self.pose is None
                or home is None
                or math.dist(self.pose, home) > float(self.get_parameter("home_tolerance_m").value)):
            return
        if self.home_handle is not None:
            self.home_handle.cancel_goal_async()
        self.homing = "back at the entrance"
        self.get_logger().info("Back at the entrance; stopped")

    def _advance_action(self) -> None:
        now = time.monotonic()
        if self.phase in {"ASSISTING", "EXTINGUISHING"} and now >= self.action_until:
            self._finish_action()
        if self.phase == "REPORTING":
            if now >= self.write_deadline:
                beacon_id = self.pending_write.beacon_id if self.pending_write else 0
                self.pending_write = None
                self._target_done(f"target handled; B{beacon_id:03d} did not confirm the update in time")
            elif now - self.last_write_sent >= 1.0:
                self._send_write()

    def _detail(self) -> str:
        parts = [f"{len(self.memories)} inherited memories"]
        if self.mission is not None and self.target is not None and not self.complete:
            order = f" ({self.target_index + 1}/{len(self.targets)})" if len(self.targets) > 1 else ""
            parts.append(f"target{order} {EVENT_NAMES.get(self.target.event_type, 'EVENT')} B{self.target.beacon_id:03d}")
        if self.route_labels and not self.complete and not self.failed and not self.phase:
            parts.append("next " + " -> ".join(label for label in self.route_labels[:3] if label))
        if self.phase == "REPORTING":
            parts.append("writing RESOLVED into the beacon")
        if self.complete and self.completion_note:
            parts.append(self.completion_note)
        if self.complete and self.homing:
            parts.append(self.homing)
        if self.stamped_hazards:
            count = len(self.stamped_hazards)
            parts.append(f"{count} hazard{'s' if count != 1 else ''} kept out")
        if self.failed and self.failure_reason:
            parts.append(self.failure_reason)
        if self.seed_ready_mission_id is None:
            parts.append("awaiting brief")
        return "; ".join(parts)

    def tick(self) -> None:
        self.update_pose()
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_tick_s is not None:
            self.battery.update(max(0.0, now - self.last_tick_s), self.pose)
        self.last_tick_s = now
        if self.battery.depleted and self.mission is not None and not self.complete and not self.failed:
            self._fail("battery depleted")
        if self.inherited_map is not None:
            self._prepare_inherited_map()
        if self.pose is not None and self.home is None:
            self.home = self.pose  # where the robot waits at the entrance
        if self.phase and not self.failed:
            self._advance_action()
        self._arrive_if_close()
        self._pass_waypoint_if_close()
        self._stop_if_home()

        status = RobotStatus()
        status.robot_id = self.robot_id
        if self.complete:
            status.state = "MISSION_COMPLETE"
        elif self.failed:
            status.state = "MISSION_FAILED"
        elif self.phase:
            status.state = self.phase
        elif self.active or self.mission is not None:
            status.state = "NAVIGATING" if self.active else "MISSION_READY"
        else:
            status.state = "READY"

        status.heartbeat = self.get_clock().now().to_msg()
        status.pose.header.frame_id = "map"
        status.pose.header.stamp = status.heartbeat
        if self.pose is not None:
            status.pose.pose.position.x = self.pose[0]
            status.pose.pose.position.y = self.pose[1]
            status.pose.pose.orientation.w = 1.0
        status.detail = self._detail()[:240]
        status.battery_percent = float(self.battery.level)
        self.status_pub.publish(status)

        if self.mission is not None and not self.active and not self.complete and not self.failed and not self.phase:
            self.maybe_plan()


def main() -> None:
    rclpy.init()
    node = Executor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
