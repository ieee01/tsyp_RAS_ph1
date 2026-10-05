from __future__ import annotations

from dataclasses import replace
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ByteMultiArray, Float32

from living_map_interfaces.msg import BeaconMemory, DetectedEvent, RobotStatus
from living_map_interfaces.srv import DeployBeacon, ResetSystem

from .aging import ttl_for
from .placement import PlacementConfig, PlacementPolicy, ring_openings
from .protocol import (FLAG_RESOLVED, FLAG_STANDOFF, BeaconPacket, apply_resolve, decode, encode, guidance,
                       place_flags, roles, target_position)
from .store import BeaconStore


class BeaconManager(Node):
    """Writer beacon dispenser plus the persistent beacons it has dropped.

    Deposition decisions use only what the Writer itself knows: its pose, its
    own SLAM map and the signal strength its radio measures. Deployed beacons
    persist (crash-tolerant store) and keep advertising independently of the
    Writer's fate.
    """

    def __init__(self) -> None:
        super().__init__("beacons", namespace="/living_map")
        self.declare_parameter("store_path", "runtime/beacons.json")
        self.declare_parameter("ttl_sec", 1800)
        self.declare_parameter("beacon_capacity", 30)
        self.declare_parameter("event_reserve", 4)
        self.declare_parameter("navigation_spacing_m", 6.0)
        self.declare_parameter("junction_separation_m", 2.5)
        self.declare_parameter("relay_threshold", 0.85)
        self.declare_parameter("standoff_min_m", 0.3)
        # An event seen next to an existing navigation beacon is written into it.
        self.declare_parameter("reuse_radius_m", 1.0)
        # Map-frame x of the mine entrance: behind it the gateway already covers.
        self.declare_parameter("entrance_x", -1.0)

        self.store = BeaconStore(str(self.get_parameter("store_path").value))
        self.items = self.store.load()
        self.seq = max((packet.sequence for packet in self.items.values()), default=0)
        self.prev = max(self.items.keys(), default=0)
        self.policy = PlacementPolicy(PlacementConfig(
            capacity=int(self.get_parameter("beacon_capacity").value),
            event_reserve=int(self.get_parameter("event_reserve").value),
            spacing_m=float(self.get_parameter("navigation_spacing_m").value),
            junction_separation_m=float(self.get_parameter("junction_separation_m").value),
            relay_threshold=float(self.get_parameter("relay_threshold").value),
        ))
        self.writer_pose: tuple[float, float] | None = None
        self.writer_state = "UNKNOWN"
        self.uplink: float | None = None
        self.map: OccupancyGrid | None = None

        self.packet_pub = self.create_publisher(ByteMultiArray, "inside/rf_tx", 20)
        self.memory_pub = self.create_publisher(BeaconMemory, "inside/beacon_memory", 20)
        self.create_service(DeployBeacon, "deploy_beacon", self.on_deploy_service)
        self.create_service(ResetSystem, "reset", self.on_reset_service)
        self.create_subscription(DetectedEvent, "/writer/detected_event", self.on_event, 10)
        # Writer's own on-board status (map-frame pose), its radio's measured
        # uplink quality and its SLAM map drive the drop decisions.
        self.create_subscription(RobotStatus, "inside/writer_status", self.on_writer_status, 10)
        self.create_subscription(Float32, "radio/writer_link", self.on_writer_link, 10)
        # Resolve requests from rescue robots, delivered by the radio.
        self.create_subscription(ByteMultiArray, "inside/beacon_write_rx", self.on_beacon_write, 20)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_timer(3.0, self.rebroadcast_all)

        recovery = " (recovered from backup)" if self.store.recovered_from_backup else ""
        self.get_logger().info(
            f"Persistent beacon manager ready; restored {len(self.items)} memories{recovery}; "
            f"dispenser holds {self.policy.remaining(len(self.items))} beacons"
        )

    # -- deployment -------------------------------------------------------
    def _duplicate(self, event_type: int, x: float, y: float, radius: float = 0.75) -> bool:
        return any(
            packet.event_type == event_type and math.dist(target_position(packet), (x, y)) < radius
            for packet in self.items.values()
        )

    def _store_and_send(self, packet: BeaconPacket) -> None:
        self.items[packet.beacon_id] = packet
        self.store.save(self.items)
        self._publish_frame(encode(packet))

    def _link_predecessor(self, previous_id: int, new_packet: BeaconPacket) -> None:
        """The previous beacon hears its successor and records the way on."""
        previous = self.items.get(previous_id)
        if previous is None:
            return
        update = {"next_beacon_id": new_packet.beacon_id}
        if not previous.flags & FLAG_STANDOFF:
            bearing, distance = guidance((previous.x_m, previous.y_m), (new_packet.x_m, new_packet.y_m))
            update.update(bearing_rad=bearing, range_m=distance)
        self.seq += 1
        self._store_and_send(replace(previous, sequence=self.seq, **update))

    def _deploy(
        self,
        x: float,
        y: float,
        event_type: int = 0,
        severity: int = 0,
        confidence: float = 1.0,
        label: str = "NAV",
        flags: int = 0,
        target: tuple[float, float] | None = None,
        reason: str = "",
    ) -> int | None:
        tx, ty = target if target is not None else (x, y)
        if event_type != 0 and self._duplicate(event_type, tx, ty):
            return None
        bearing, distance = 0.0, 0.0
        if target is not None and math.dist((x, y), target) >= float(self.get_parameter("standoff_min_m").value):
            bearing, distance = guidance((x, y), target)
            flags |= FLAG_STANDOFF
        else:
            x, y = tx, ty

        # Record the kind of place the beacon sits in (junction / corridor / dead end).
        flags |= place_flags(self._openings((x, y)))
        self.seq += 1
        beacon_id = max(self.items.keys(), default=0) + 1
        packet = BeaconPacket(
            flags=flags,
            mission_id=1,
            beacon_id=beacon_id,
            sequence=self.seq,
            timestamp_sec=int(time.time()),
            x_m=x,
            y_m=y,
            bearing_rad=bearing,
            range_m=distance,
            event_type=event_type,
            severity=severity,
            confidence=confidence,
            previous_beacon_id=self.prev,
            ttl_sec=(
                min(int(self.get_parameter("ttl_sec").value), ttl_for(event_type))
                if event_type == 0 else ttl_for(event_type)
            ),
        )
        previous_id = self.prev
        self.prev = beacon_id
        self._store_and_send(packet)
        self._link_predecessor(previous_id, packet)
        self._publish_memory(packet, label)
        role = "/".join(roles(packet.flags)) or "NAV"
        where = f" -> {label} {distance:.1f} m at {math.degrees(bearing):.0f} deg" if packet.flags & FLAG_STANDOFF else ""
        self.get_logger().info(
            f"Deployed B{beacon_id:03d} [{role}] type={label} at map=({x:.2f},{y:.2f}){where}"
            f"{f' ({reason})' if reason else ''}; {self.policy.remaining(len(self.items))} beacons left"
        )
        return beacon_id

    def _publish_memory(self, packet: BeaconPacket, label: str) -> None:
        memory = BeaconMemory()
        memory.protocol_version = packet.protocol_version
        memory.mission_id = packet.mission_id
        memory.beacon_id = packet.beacon_id
        memory.sequence = packet.sequence
        memory.written_at.sec = packet.timestamp_sec
        memory.x = packet.x_m
        memory.y = packet.y_m
        memory.yaw = packet.bearing_rad
        memory.range_m = packet.range_m
        memory.flags = packet.flags
        memory.event_type = packet.event_type
        memory.severity = packet.severity
        memory.confidence = packet.confidence
        memory.previous_beacon_id = packet.previous_beacon_id
        memory.suggested_next_beacon_id = packet.next_beacon_id
        memory.ttl_seconds = packet.ttl_sec
        memory.crc32 = int.from_bytes(encode(packet)[-4:], "big")
        memory.age_seconds = 0.0
        memory.freshness = "FRESH"
        memory.event_id = label
        self.memory_pub.publish(memory)

    def _publish_frame(self, frame: bytes) -> None:
        msg = ByteMultiArray()
        # Jazzy's generated Python converter for uint8[] expects a bytes-like
        # value. Assigning list(frame) aborts in the C conversion layer.
        msg.data = frame
        self.packet_pub.publish(msg)

    # -- inputs -----------------------------------------------------------
    def on_deploy_service(self, request: DeployBeacon.Request, response: DeployBeacon.Response) -> DeployBeacon.Response:
        event = request.event
        beacon_id = self._deploy_event(event)
        response.success = beacon_id is not None
        response.beacon_id = int(beacon_id or 0)
        response.message = "deployed" if beacon_id is not None else "duplicate suppressed or dispenser empty"
        return response

    def on_reset_service(self, request: ResetSystem.Request, response: ResetSystem.Response) -> ResetSystem.Response:
        if request.clear_persistent_memory:
            self.items.clear()
            self.prev = 0
            self.seq = 0
            self.store.save(self.items)
        response.success = True
        response.message = (
            "persistent beacon memory cleared" if request.clear_persistent_memory else "no persistent data cleared"
        )
        return response

    def _deploy_event(self, event: DetectedEvent) -> int | None:
        if not self.policy.can_deploy_event(len(self.items)):
            self.get_logger().error(f"Dispenser empty: cannot record {event.event_id}")
            return None
        target = (event.pose.pose.position.x, event.pose.pose.position.y)
        if self.writer_pose is None or self.writer_state == "FAILED":
            # No reliable Writer pose: record the event where it was sensed.
            return self._deploy(*target, event.event_type, event.severity, event.confidence, event.event_id or "EVENT")
        if self._duplicate(event.event_type, *target):
            return None
        reused = self._reuse_navigation_beacon(event, target)
        if reused is not None:
            return reused
        return self._deploy(*self.writer_pose, event.event_type, event.severity, event.confidence,
                            event.event_id or "EVENT", target=target, reason="safe standoff")

    def _reuse_navigation_beacon(self, event: DetectedEvent, target: tuple[float, float]) -> int | None:
        """Write an event into a navigation beacon the Writer is standing next to.

        Saves a beacon from the limited dispenser: the existing beacon keeps its
        roles and chain links and gains the event plus a guidance vector to it.
        """
        radius = float(self.get_parameter("reuse_radius_m").value)
        nearby = [p for p in self.items.values()
                  if p.event_type == 0 and math.dist((p.x_m, p.y_m), self.writer_pose) <= radius]
        if not nearby:
            return None
        beacon = min(nearby, key=lambda p: math.dist((p.x_m, p.y_m), self.writer_pose))
        bearing, distance = guidance((beacon.x_m, beacon.y_m), target)
        self.seq += 1
        updated = replace(
            beacon,
            flags=beacon.flags | FLAG_STANDOFF,
            sequence=self.seq,
            timestamp_sec=int(time.time()),
            bearing_rad=bearing,
            range_m=distance,
            event_type=event.event_type,
            severity=event.severity,
            confidence=event.confidence,
            ttl_sec=ttl_for(event.event_type),
        )
        self._store_and_send(updated)
        label = event.event_id or "EVENT"
        self._publish_memory(updated, label)
        self.get_logger().info(
            f"Rewrote B{beacon.beacon_id:03d} [{'/'.join(roles(updated.flags))}] to record {label} "
            f"{distance:.1f} m at {math.degrees(bearing):.0f} deg (beacon reused; "
            f"{self.policy.remaining(len(self.items))} beacons left)"
        )
        return beacon.beacon_id

    def on_event(self, event: DetectedEvent) -> None:
        self._deploy_event(event)

    def on_writer_link(self, msg: Float32) -> None:
        self.uplink = float(msg.data)

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def _openings(self, pose: tuple[float, float]) -> int:
        if self.map is None:
            return 0
        info = self.map.info
        return ring_openings(self.map.data, info.width, info.height, info.resolution,
                             (info.origin.position.x, info.origin.position.y), pose)

    def on_writer_status(self, status: RobotStatus) -> None:
        if status.robot_id != "writer" or status.pose.header.frame_id != "map":
            return
        self.writer_state = status.state
        self.writer_pose = (status.pose.pose.position.x, status.pose.pose.position.y)
        # Only a moving, exploring Writer inside the mine drops navigation beacons.
        if status.state not in {"ONLINE", "RETURNING"}:
            return
        if self.writer_pose[0] < float(self.get_parameter("entrance_x").value):
            return
        decision = self.policy.decide(
            self.writer_pose,
            [(p.x_m, p.y_m) for p in self.items.values()],
            len(self.items),
            self.uplink,
            self._openings(self.writer_pose),
            time.monotonic(),
        )
        if decision is not None:
            self._deploy(*self.writer_pose, flags=decision.flags, reason=decision.reason)

    def on_beacon_write(self, msg: ByteMultiArray) -> None:
        """A robot marks an event resolved: the beacon updates its own memory."""
        try:
            try:
                raw = bytes(msg.data)
            except TypeError:  # some rmw layers deliver uint8[] as one-byte objects
                raw = b"".join(v if isinstance(v, bytes) else bytes((v,)) for v in msg.data)
            request = decode(raw)
        except Exception as exc:
            self.get_logger().warning(f"Rejected beacon write: {exc}")
            return
        stored = self.items.get(request.beacon_id)
        if stored is None:
            return
        if stored.flags & FLAG_RESOLVED:
            self._publish_frame(encode(stored))  # Already resolved: re-advertise as the acknowledgement.
            return
        self.seq = max(self.seq, request.sequence, stored.sequence) + 1
        updated = apply_resolve(stored, request, self.seq)
        if updated is None:
            self.get_logger().warning(f"Rejected write to B{request.beacon_id:03d}: not a resolve of that memory")
            self.seq -= 1
            return
        self._store_and_send(updated)
        self.get_logger().info(f"B{updated.beacon_id:03d} marked RESOLVED by a rescue robot (seq {updated.sequence})")

    def rebroadcast_all(self) -> None:
        # Models physical beacon persistence / periodic advertising independently
        # of Writer lifetime.
        for packet in list(self.items.values()):
            self._publish_frame(encode(packet))


def main() -> None:
    rclpy.init()
    node = BeaconManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
