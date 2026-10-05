from __future__ import annotations

import json
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ByteMultiArray, Float32, Float32MultiArray, String, UInt16

from living_map_beacons.protocol import decode
from living_map_interfaces.msg import RobotStatus

from .mesh import GATEWAY, RadioMesh
from .model import DEFAULT_WALL_LOSS_DB, RFModel
from .walls import load_walls


def _frame_bytes(data) -> bytes:
    # Some Jazzy/rmw combinations deserialize uint8[] as a sequence of
    # one-byte objects instead of integers. Accept both representations.
    try:
        return bytes(data)
    except TypeError:
        return b"".join(value if isinstance(value, bytes) else bytes((value,)) for value in data)


class Radio(Node):
    """The radio channel of the disconnected mine.

    It is the only path between the inside (Writer, Executor, beacons) and the
    Outside Network gateway. Beacon frames are flooded through the beacon
    mesh; robot telemetry is routed along the best mesh route. Anything with
    no radio route simply does not arrive.
    """

    def __init__(self) -> None:
        super().__init__("radio", namespace="/living_map")
        self.declare_parameter("gateway_x", -1.5)
        self.declare_parameter("gateway_y", 0.0)
        self.declare_parameter("rf_config", "config/rf.yaml")
        self.declare_parameter("world_file", "")
        self.declare_parameter("map_period_sec", 2.0)
        # Rescue robots on the radio besides the Writer (each has its own topics).
        self.declare_parameter("rescue_robots", ["executor"])

        config_path = Path(str(self.get_parameter("rf_config").value))
        try:
            config = yaml.safe_load(config_path.read_text()) or {}
        except Exception as exc:
            self.get_logger().warning(f"RF config load failed, using defaults: {exc}")
            config = {}

        walls = []
        world_file = str(self.get_parameter("world_file").value)
        if world_file:
            try:
                walls = load_walls(world_file)
            except Exception as exc:
                self.get_logger().error(f"World walls unavailable ({exc}); RF model is free-space only")
        model = RFModel(
            int(config.get("seed", 42)),
            float(config.get("nominal_range_m", 12.0)),
            float(config.get("max_range_m", 22.0)),
            walls,
            float(config.get("wall_loss_db", DEFAULT_WALL_LOSS_DB)),
        )
        gateway = (float(self.get_parameter("gateway_x").value), float(self.get_parameter("gateway_y").value))
        self.mesh = RadioMesh(
            model,
            gateway,
            float(config.get("min_link_probability", 0.3)),
            int(config.get("link_retries", 3)),
            int(config.get("max_hops", 16)),
        )
        self.routes = {}
        self.robots: dict[str, dict] = {}
        self.latest_map: OccupancyGrid | None = None

        self.gateway_pub = self.create_publisher(ByteMultiArray, "gateway/rf_rx", 20)
        self.metrics_pub = self.create_publisher(Float32MultiArray, "radio/metrics", 20)
        self.status_pub = self.create_publisher(RobotStatus, "gateway/robot_status", 20)
        self.confirm_pub = self.create_publisher(UInt16, "gateway/beacon_confirmed", 20)
        self.beacon_write_pub = self.create_publisher(ByteMultiArray, "inside/beacon_write_rx", 20)
        self.topology_pub = self.create_publisher(String, "radio/topology", 5)
        self.writer_link_pub = self.create_publisher(Float32, "radio/writer_link", 5)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, "gateway/map_rx", latched)

        self.create_subscription(ByteMultiArray, "inside/rf_tx", self.on_frame, 50)
        self.create_subscription(RobotStatus, "inside/writer_status", self.on_robot_status, 20)
        self.rescue_robots = [str(name) for name in self.get_parameter("rescue_robots").value]
        self.hearing_pubs = {}
        for robot in self.rescue_robots:
            # What each rescue robot hears directly from nearby beacons.
            self.hearing_pubs[robot] = self.create_publisher(ByteMultiArray, f"{robot}/rf_rx", 20)
            self.create_subscription(RobotStatus, f"inside/{robot}_status", self.on_robot_status, 20)
            self.create_subscription(UInt16, f"inside/{robot}_beacon_confirmed",
                                     lambda msg, r=robot: self.on_confirmation(r, msg), 20)
            self.create_subscription(ByteMultiArray, f"inside/{robot}_beacon_write",
                                     lambda msg, r=robot: self.on_beacon_write(r, msg), 20)
        self.create_subscription(UInt16, "radio/disable_beacon", self.on_disable_beacon, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_timer(1.0, self.publish_topology)
        self.create_timer(float(self.get_parameter("map_period_sec").value), self.forward_map)
        self.get_logger().info(
            f"Radio mesh ready: {len(walls)} RF-blocking walls, "
            f"{model.wall_loss_db:.0f} dB per wall, gateway at ({gateway[0]:.1f},{gateway[1]:.1f})"
        )

    # -- helpers ----------------------------------------------------------
    def _refresh_routes(self) -> None:
        self.routes = self.mesh.routes()

    def _robot_point(self, robot_id: str):
        robot = self.robots.get(robot_id)
        return None if robot is None else robot["point"]

    def _metric(self, beacon_id: int, quality, delivered: bool, destination: str, hops: int) -> None:
        metric = Float32MultiArray()
        metric.data = [
            float(beacon_id),
            float(quality.distance_m),
            float(quality.probability),
            float(quality.rssi_dbm),
            1.0 if delivered else 0.0,
            0.0 if destination == "gateway" else 1.0,
            float(hops),
            float(quality.walls),
        ]
        self.metrics_pub.publish(metric)

    # -- beacon frames ----------------------------------------------------
    def on_frame(self, msg: ByteMultiArray) -> None:
        frame = _frame_bytes(msg.data)
        try:
            packet = decode(frame)
        except Exception as exc:
            self.get_logger().warning(f"Drop invalid frame: {exc}")
            return
        if packet.beacon_id in self.mesh.dead:
            return
        if self.mesh.update_beacon(packet.beacon_id, (packet.x_m, packet.y_m)):
            self._refresh_routes()

        result = self.mesh.flood(packet.beacon_id)
        gateway_attempts = [a for a in result.attempts if a[1] == GATEWAY]
        if gateway_attempts:
            _, _, quality, delivered = gateway_attempts[-1]
            self._metric(packet.beacon_id, quality, delivered, "gateway", result.gateway_hops or 0)
        if result.delivered:
            self.gateway_pub.publish(msg)
            if result.gateway_hops and result.gateway_hops > 1:
                self.get_logger().debug(
                    f"B{packet.beacon_id:03d} relayed in {result.gateway_hops} hops via "
                    + " -> ".join(f"B{b:03d}" for b in result.gateway_path)
                )

        # Rescue robots hear beacons directly, by being physically near them.
        for robot, publisher in self.hearing_pubs.items():
            point = self._robot_point(robot)
            if point is None:
                continue
            quality = self.mesh.model.link((packet.x_m, packet.y_m), point)
            delivered = self.mesh.model.sample(quality.probability)
            self._metric(packet.beacon_id, quality, delivered, robot, 1)
            if delivered:
                publisher.publish(msg)

    def on_disable_beacon(self, msg: UInt16) -> None:
        beacon_id = int(msg.data)
        self.mesh.kill(beacon_id)
        self._refresh_routes()
        self.get_logger().error(f"FAULT INJECTED: beacon B{beacon_id:03d} destroyed; mesh re-routing")

    # -- robot telemetry --------------------------------------------------
    def on_robot_status(self, msg: RobotStatus) -> None:
        if msg.pose.header.frame_id != "map":
            return
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robots[msg.robot_id] = {"point": point, "state": msg.state}
        if msg.robot_id == "writer" and msg.state != "FAILED":
            # What the Writer's own radio measures: its best link into the mesh.
            self.writer_link_pub.publish(Float32(data=float(self.mesh.uplink_quality(point, self.routes))))
        delivered, _hops = self.mesh.send_unicast(point, self.routes)
        if delivered:
            self.status_pub.publish(msg)

    def on_confirmation(self, robot: str, msg: UInt16) -> None:
        point = self._robot_point(robot)
        if point is not None and self.mesh.send_unicast(point, self.routes)[0]:
            self.confirm_pub.publish(msg)

    def on_beacon_write(self, robot: str, msg: ByteMultiArray) -> None:
        """A robot updates the beacon next to it over a direct radio link."""
        point = self._robot_point(robot)
        try:
            request = decode(_frame_bytes(msg.data))
        except Exception as exc:
            self.get_logger().warning(f"Drop invalid beacon write from {robot}: {exc}")
            return
        target = self.mesh.beacons.get(request.beacon_id)
        if point is None or target is None or request.beacon_id in self.mesh.dead:
            return
        probability = self.mesh.model.link(point, target).probability
        if any(self.mesh.model.sample(probability) for _ in range(self.mesh.link_retries)):
            self.beacon_write_pub.publish(msg)

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def forward_map(self) -> None:
        """The Writer uploads its SLAM map while it is alive and has a route out."""
        writer = self.robots.get("writer")
        if self.latest_map is None or writer is None or writer["state"] == "FAILED":
            return
        if self.mesh.send_unicast(writer["point"], self.routes)[0]:
            self.map_pub.publish(self.latest_map)

    def publish_topology(self) -> None:
        self._refresh_routes()
        robots = {name: robot["point"] for name, robot in self.robots.items()}
        topology = self.mesh.topology(robots)
        self.topology_pub.publish(String(data=json.dumps(topology, separators=(",", ":"))))


def main() -> None:
    rclpy.init()
    node = Radio()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
