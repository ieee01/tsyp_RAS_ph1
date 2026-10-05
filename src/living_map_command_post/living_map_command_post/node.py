from __future__ import annotations

import json
import os
import threading
import queue
import subprocess
import sys
import time
from pathlib import Path

import yaml

from .demo import DemoController, select_target

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .state import CommandPostState
from .scenario import prepare_scenario, scenario_layout


class CommandPost(Node):
    """Independent browser/API process beyond the simulated long-distance link."""

    def __init__(self) -> None:
        super().__init__("command_post", namespace="/living_map")
        self.state = CommandPostState()
        self.mission_pub = self.create_publisher(String, "link/command_post_tx", 20)
        self.create_subscription(String, "command_post/link_rx", self.on_snapshot, 50)
        self.outbox = queue.SimpleQueue()
        self.declare_parameter("events_file", "")
        events_file = Path(str(self.get_parameter("events_file").value))
        gas_count = 1
        if events_file.is_file():
            events = yaml.safe_load(events_file.read_text())["events"]
            gas_count = sum(event["type"] == "GAS_HAZARD" for event in events)
        self.demo = DemoController(self.state, self.outbox.put, os.environ.get("LIVING_MAP_DEMO_MODE", "autonomous"),
                                   speed=os.environ.get("LIVING_MAP_SPEED", "normal"),
                                   min_gas_hazards=max(1, gas_count))
        # Simulation harness only: the outage switch of the simulated satellite link.
        self.link_client = self.create_client(SetBool, "link/set_enabled")
        self.link_restore_at: float | None = None
        self.create_timer(0.2, self.tick)
        self.restarting = False
        self.start_http()

    def set_link(self, up: bool, restore_after_s: float | None = None) -> bool:
        if not self.link_client.service_is_ready():
            return False
        self.link_client.call_async(SetBool.Request(data=bool(up)))
        self.link_restore_at = None if up or not restore_after_s else time.monotonic() + float(restore_after_s)
        self.demo.note("Harness: satellite link restored" if up else
                       f"Harness: satellite link cut{f' for {restore_after_s:.0f} s' if restore_after_s else ''}")
        return True

    def tick(self):
        if self.link_restore_at is not None and time.monotonic() >= self.link_restore_at:
            self.set_link(True)
        self.demo.tick()
        while True:
            try:
                payload = self.outbox.get_nowait()
            except queue.Empty:
                break
            self.mission_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def request_restart(self, mode, speed="normal", events=None):
        """Apply optional event positions and start the local simulation replay."""
        root = Path(os.environ.get("LIVING_MAP_ROOT", ""))
        helper = root / "tools" / "restart_demo.py"
        if self.restarting or not helper.is_file():
            return False
        if events is not None:
            prepare_scenario(root, events)
        self.restarting = True
        log = (root / "runtime" / "restart.log").open("a")
        subprocess.Popen([sys.executable, str(helper), mode, speed], cwd=root,
                         stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                         start_new_session=True)
        log.close()
        return True

    def on_snapshot(self, msg: String) -> None:
        try:
            self.state.replace(json.loads(msg.data))
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejected malformed gateway snapshot: {exc}")

    def request_mission(self, payload: dict) -> bool:
        """Send a WHO/WHAT mission through the uplink; the gateway validates it."""
        payload = dict(payload)
        if not payload.get("target_beacon_id"):
            target = select_target(self.state.snapshot(), int(payload.get("target_type", 1)),
                                   int(payload.get("mission_id", 1)))
            if target is None:
                raise ValueError("No live memory of the requested type to target")
            payload["target_beacon_id"] = int(target["beacon_id"])
        payload.pop("target_type", None)
        self.outbox.put(payload)
        self.state.mark_mission_sent(int(payload.get("mission_id", 1)))
        return True

    def start_http(self) -> None:
        import uvicorn
        from .web import create_app
        host = os.environ.get("LIVING_MAP_COMMAND_POST_HOST", "127.0.0.1")
        port = int(os.environ.get("LIVING_MAP_COMMAND_POST_PORT", "8081"))
        app = create_app(self.state, self.request_mission, self.demo, self.request_restart,
                         lambda: scenario_layout(Path(os.environ.get("LIVING_MAP_ROOT", ""))), self.set_link)
        thread = threading.Thread(
            target=lambda: uvicorn.run(app, host=host, port=port, log_level="warning"),
            daemon=True,
            name="living_map_command_post_http",
        )
        thread.start()
        self.get_logger().info(f"Command Post available on http://localhost:{port}")


def main() -> None:
    rclpy.init()
    node = CommandPost()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
