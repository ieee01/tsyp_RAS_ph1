from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from living_map_interfaces.msg import DetectedEvent


EVENT_TYPES = {"VICTIM": 1, "GAS_HAZARD": 2, "BLOCKED_PATH": 3, "FIRE": 4}


class EventSensor(Node):
    """Deterministic Phase-1 event sensor operating in the Writer SLAM map frame."""

    def __init__(self) -> None:
        super().__init__("event_sensor", namespace="/living_map")
        self.declare_parameter("events_file", "config/events.yaml")
        events_file = Path(str(self.get_parameter("events_file").value))
        self.events = yaml.safe_load(events_file.read_text())["events"]
        self.seen: set[str] = set()
        self.markers: list[Marker] = []
        self.odom_fallback: tuple[float, float] | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(DetectedEvent, "/writer/detected_event", 10)
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, "event_markers", marker_qos)
        self.create_subscription(Odometry, "/writer/odom", self.on_odom, 10)
        # Simulation harness: a rescue robot put a fire out.
        self.declare_parameter("world_name", "living_map_mine")
        self.create_subscription(String, "sim/extinguish", self.on_extinguish, 10)
        self.extinguished: set[str] = set()
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f"Loaded {len(self.events)} simulated events from {events_file}")

    def on_odom(self, msg: Odometry) -> None:
        self.odom_fallback = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def event_map_pose(self, event: dict) -> tuple[float, float]:
        """Transform an event configured in Writer-local odometry into SLAM map."""
        try:
            transform = self.tf_buffer.lookup_transform("map", "writer/odom", Time())
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            local_x, local_y = float(event["x"]), float(event["y"])
            return (
                transform.transform.translation.x
                + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
                transform.transform.translation.y
                + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
            )
        except TransformException:
            return float(event["x"]), float(event["y"])

    def on_extinguish(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
            x, y = float(request["x"]), float(request["y"])
        except (TypeError, ValueError, KeyError) as exc:
            self.get_logger().warning(f"Ignoring malformed extinguish request: {exc}")
            return
        fires = [e for e in self.events if e["type"] == "FIRE" and e["id"] not in self.extinguished]
        if not fires:
            return
        fire = min(fires, key=lambda e: math.dist(self.event_map_pose(e), (x, y)))
        if math.dist(self.event_map_pose(fire), (x, y)) > 1.5:
            self.get_logger().warning(f"No fire near ({x:.1f}, {y:.1f}) to extinguish")
            return
        self.extinguished.add(fire["id"])
        self.seen.add(fire["id"])  # An extinguished fire is never detected again.
        model = f"fire_{fire['id']}"
        world = str(self.get_parameter("world_name").value)
        # Remove the flames from the simulated world (non-blocking).
        subprocess.Popen(
            ["gz", "service", "-s", f"/world/{world}/remove", "--reqtype", "gz.msgs.Entity",
             "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f'name: "{model}" type: MODEL'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for marker in self.markers:
            if marker.ns == "events" and abs(marker.pose.position.x - self.event_map_pose(fire)[0]) < 0.01:
                marker.color.r = marker.color.g = marker.color.b = 0.45
        array = MarkerArray()
        array.markers = list(self.markers)
        self.marker_pub.publish(array)
        self.get_logger().info(f"FIRE {fire['id']} extinguished by a rescue robot; removed {model} from the world")

    def tick(self) -> None:
        # Detection zones remain in stable Writer-local odometry coordinates.
        pose = self.odom_fallback
        if pose is None:
            return
        x, y = pose
        for event in self.events:
            if event["id"] in self.seen:
                continue
            if math.hypot(x - float(event["x"]), y - float(event["y"])) > float(event["detection_radius"]):
                continue
            self.seen.add(event["id"])
            event_x, event_y = self.event_map_pose(event)
            msg = DetectedEvent()
            msg.event_id = event["id"]
            msg.event_type = EVENT_TYPES[event["type"]]
            msg.pose.header.frame_id = "map"
            msg.pose.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = event_x
            msg.pose.pose.position.y = event_y
            msg.pose.pose.orientation.w = 1.0
            msg.severity = int(event["severity"])
            msg.confidence = float(event.get("confidence", 0.95))
            msg.detected_at = msg.pose.header.stamp
            self.pub.publish(msg)
            self.add_marker(event, msg.event_type, event_x, event_y)
            self.get_logger().info(
                f"{event['type']} {event['id']} detected confidence={msg.confidence:.2f} "
                f"at map=({event_x:.2f},{event_y:.2f})"
            )

    def add_marker(self, event: dict, event_type: int, event_x: float, event_y: float) -> None:
        marker_id = len(self.markers) + 1
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "events"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = event_x
        marker.pose.position.y = event_y
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.35
        marker.color.a = 1.0
        marker.color.r, marker.color.g = {1: (0.15, 1.0), 4: (1.0, 0.55)}.get(event_type, (1.0, 0.15))
        self.markers.append(marker)

        label = Marker()
        label.header.frame_id = "map"
        label.header.stamp = marker.header.stamp
        label.ns = "event_labels"
        label.id = 100 + marker_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = event_x
        label.pose.position.y = event_y
        label.pose.position.z = 0.55
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = event["id"]
        self.markers.append(label)

        array = MarkerArray()
        array.markers = list(self.markers)
        self.marker_pub.publish(array)


def main() -> None:
    rclpy.init()
    node = EventSensor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
