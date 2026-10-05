#!/usr/bin/env python3
"""Restart Gazebo if it freezes at start-up.

Occasionally `gz sim` hangs before loading the world: no physics, no clock, no
robots. This guard waits for the first simulation clock message. If none
arrives within ``timeout_s`` it kills the frozen Gazebo process tree; the launch
file respawns Gazebo and the ROS bridge reconnects by itself. Once the clock
ticks, the guard's job is done.
"""
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimGuard(Node):
    def __init__(self) -> None:
        super().__init__("sim_guard", namespace="/living_map")
        self.declare_parameter("timeout_s", 40.0)
        self.declare_parameter("max_restarts", 3)
        self.ticked = False
        self.create_subscription(Clock, "/clock_raw", self.on_clock,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_clock(self, _msg: Clock) -> None:
        self.ticked = True


def gazebo_pids() -> list[int]:
    """The `gz sim -r …` launcher and its server / GUI children."""
    out = subprocess.run(["pgrep", "-f", "^gz sim( -r .*| server| gui)$"], capture_output=True, text=True).stdout
    return [int(pid) for pid in out.split()]


def main() -> None:
    rclpy.init()
    node = SimGuard()
    timeout = float(node.get_parameter("timeout_s").value)
    restarts = 0
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.ticked:
                node.get_logger().info("Simulation clock is running; guard done")
                break
            if time.monotonic() - started < timeout:
                continue
            if restarts >= int(node.get_parameter("max_restarts").value):
                node.get_logger().error("Gazebo did not start after several attempts; giving up")
                break
            restarts += 1
            node.get_logger().warning(f"No simulation clock after {timeout:.0f} s: Gazebo is frozen, restarting it "
                                      f"(attempt {restarts})")
            for pid in gazebo_pids():
                try:
                    subprocess.run(["kill", f"-{signal.SIGKILL.value}", str(pid)], check=False)
                except OSError:
                    pass
            started = time.monotonic()
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
