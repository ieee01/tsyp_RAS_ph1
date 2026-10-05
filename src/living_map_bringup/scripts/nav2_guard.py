#!/usr/bin/env python3
"""Finish a Nav2 bringup that stalled.

Under heavy load a lifecycle service reply can be lost: the node becomes active
but its lifecycle manager never hears it and blocks forever, leaving the later
servers (behaviors, BT navigator) inactive and every goal rejected. This guard
watches each robot's Nav2 nodes and, when a stack makes no progress for
``stall_s``, configures and activates the remaining nodes in bringup order.
"""
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

ORDER = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]


class Nav2Guard(Node):
    def __init__(self) -> None:
        super().__init__("nav2_guard", namespace="/living_map")
        self.declare_parameter("robots", ["writer", "executor"])
        self.declare_parameter("grace_s", 25.0)
        self.declare_parameter("stall_s", 15.0)
        self.robots = [str(r) for r in self.get_parameter("robots").value]
        self.started = time.monotonic()
        self.status: dict[str, dict] = {r: {"states": None, "since": time.monotonic(), "done": False}
                                        for r in self.robots}
        self.get_state = {(r, n): self.create_client(GetState, f"/{r}/{n}/get_state")
                          for r in self.robots for n in ORDER}
        self.change_state = {(r, n): self.create_client(ChangeState, f"/{r}/{n}/change_state")
                             for r in self.robots for n in ORDER}

    def _state(self, robot: str, name: str) -> int | None:
        client = self.get_state[(robot, name)]
        if not client.service_is_ready():
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result().current_state.id if future.done() and future.result() else None

    def _transition(self, robot: str, name: str, transition: int) -> bool:
        client = self.change_state[(robot, name)]
        future = client.call_async(ChangeState.Request(transition=Transition(id=transition)))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return bool(future.done() and future.result() and future.result().success)

    def finished(self) -> bool:
        return all(status["done"] for status in self.status.values())

    def check(self) -> None:
        if time.monotonic() - self.started < float(self.get_parameter("grace_s").value):
            return
        for robot, status in self.status.items():
            if status["done"]:
                continue
            states = tuple(self._state(robot, name) for name in ORDER)
            if all(s == State.PRIMARY_STATE_ACTIVE for s in states):
                status["done"] = True
                self.get_logger().info(f"{robot}: Nav2 stack active")
                continue
            if states != status["states"]:
                status["states"], status["since"] = states, time.monotonic()
                continue
            if time.monotonic() - status["since"] < float(self.get_parameter("stall_s").value):
                continue
            self.get_logger().warning(f"{robot}: Nav2 bringup stalled at {states}; completing it")
            for name, state in zip(ORDER, states):
                if state == State.PRIMARY_STATE_UNCONFIGURED:
                    self._transition(robot, name, Transition.TRANSITION_CONFIGURE)
                    state = State.PRIMARY_STATE_INACTIVE
                if state == State.PRIMARY_STATE_INACTIVE:
                    ok = self._transition(robot, name, Transition.TRANSITION_ACTIVATE)
                    self.get_logger().info(f"{robot}/{name} activate: {'ok' if ok else 'failed'}")
            status["since"] = time.monotonic()


def main() -> None:
    rclpy.init()
    node = Nav2Guard()
    try:
        # A plain loop: the node is only spun while waiting for a reply, never re-entrantly.
        while rclpy.ok() and not node.finished():
            node.check()
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(2.0)
        node.get_logger().info("All Nav2 stacks active; guard done")
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
