from __future__ import annotations

import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .model import LinkModel


class LinkSimulator(Node):
    def __init__(self) -> None:
        super().__init__("long_distance_link", namespace="/living_map")
        self.declare_parameter("config", "config/link.yaml")
        try:
            config = yaml.safe_load(Path(str(self.get_parameter("config").value)).read_text()) or {}
        except Exception as exc:
            self.get_logger().warning(f"Link config unavailable, using defaults: {exc}")
            config = {}
        self.model = LinkModel(
            seed=int(config.get("seed", 73)),
            latency_ms=float(config.get("latency_ms", 600)),
            jitter_ms=float(config.get("jitter_ms", 80)),
            packet_loss=float(config.get("packet_loss", 0.02)),
            bandwidth_bytes_per_sec=float(config.get("bandwidth_bytes_per_sec", 32768)),
            initially_up=bool(config.get("initially_up", True)),
        )
        self.command_post_pub = self.create_publisher(String, "command_post/link_rx", 50)
        self.gateway_pub = self.create_publisher(String, "gateway/link_rx", 20)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(Bool, "link/status", status_qos)
        self.create_subscription(String, "link/gateway_tx", self.on_downlink, 50)
        self.create_subscription(String, "link/command_post_tx", self.on_uplink, 20)
        self.create_service(SetBool, "link/set_enabled", self.on_set_enabled)
        self.create_timer(0.02, self.tick)
        self.create_timer(1.0, self.publish_status)
        self.publish_status()

    def on_downlink(self, msg: String) -> None:
        self.model.submit("downlink", msg.data, time.monotonic())

    def on_uplink(self, msg: String) -> None:
        self.model.submit("uplink", msg.data, time.monotonic())

    def on_set_enabled(self, request, response):
        self.model.set_up(request.data, time.monotonic())
        self.publish_status()
        response.success = True
        response.message = "link restored" if request.data else "link outage enabled"
        return response

    def publish_status(self) -> None:
        msg = Bool()
        msg.data = self.model.up
        self.status_pub.publish(msg)

    def tick(self) -> None:
        for delivery in self.model.ready(time.monotonic()):
            msg = String()
            msg.data = delivery.payload
            if delivery.direction == "downlink":
                self.command_post_pub.publish(msg)
            else:
                self.gateway_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = LinkSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
