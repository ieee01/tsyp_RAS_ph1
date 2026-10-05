from __future__ import annotations

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier import choose_frontier
from .sortie import Sortie


class FrontierExplorer(Node):
    """Occupancy-grid frontier explorer using Nav2 goals in the SLAM map frame.

    When no reachable frontier remains, or the sortie budget (battery reserve)
    is spent, the Writer drives back to the entrance: the best case, in which
    it hands over a complete map in person.
    """

    def __init__(self) -> None:
        super().__init__("frontier_explorer", namespace="/writer")
        self.map: OccupancyGrid | None = None
        self.odom_fallback = (0.0, 0.0)
        self.busy = False
        self.blacklist: list[tuple[float, float]] = []
        # Suppress the local area around successful goals while retaining other
        # unexplored sections of the same connected boundary.
        self.visited: list[tuple[float, float]] = []
        self.failed = False
        self.stuck = False
        self.enabled = False
        self.goal_handle = None

        self.declare_parameter("min_cluster_cells", 5)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("exploration_budget_s", 0.0)
        self.declare_parameter("empty_ticks_to_complete", 4)
        self.declare_parameter("max_return_attempts", 3)
        # Map-frame x of the mine entrance; the Writer explores only beyond it.
        self.declare_parameter("entrance_x", -1.0)
        self.sortie = Sortie(
            float(self.get_parameter("exploration_budget_s").value),
            int(self.get_parameter("empty_ticks_to_complete").value),
            int(self.get_parameter("max_return_attempts").value),
        )
        self.last_tick_s: float | None = None

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.create_subscription(Odometry, "/writer/odom", self.on_odom, 10)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/living_map/writer_failed", self.on_failed, state_qos)
        self.create_subscription(Bool, "/living_map/writer_enabled", self.on_enabled, state_qos)
        self.create_subscription(Bool, "/living_map/writer_stuck", self.on_stuck, state_qos)
        self.create_subscription(Bool, "/living_map/writer_return_request", self.on_return_request, state_qos)
        self.state_pub = self.create_publisher(String, "/living_map/writer_explorer_state", state_qos)
        self.published_state = ""
        self.create_timer(2.0, self.tick)

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def on_odom(self, msg: Odometry) -> None:
        self.odom_fallback = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def map_pose(self) -> tuple[float, float]:
        try:
            transform = self.tf_buffer.lookup_transform("map", "writer/base_link", Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except TransformException:
            return self.odom_fallback

    def _cancel(self, reason: str) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.get_logger().warning(f"Active Writer goal cancelled: {reason}")

    def on_enabled(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self._cancel("paused")

    def on_failed(self, msg: Bool) -> None:
        self.failed = bool(msg.data)
        if self.failed:
            self._cancel("Writer failure")

    def on_stuck(self, msg: Bool) -> None:
        self.stuck = bool(msg.data)
        if self.stuck:
            self._cancel("Writer immobilised")

    def on_return_request(self, msg: Bool) -> None:
        if msg.data and self.sortie.phase == "EXPLORING":
            self.sortie.start_return("battery reserve reached")
            self._cancel("battery reserve reached; heading home")
            self.get_logger().warning("Battery reserve reached: returning to the entrance")

    def _publish_state(self) -> None:
        if self.failed:
            state = "HALTED"
        elif self.stuck:
            state = "STUCK"
        elif self.sortie.phase in {"RETURNING", "HOME", "RETURN_FAILED"}:
            state = self.sortie.phase
        elif not self.enabled:
            state = "PAUSED"
        else:
            state = "NAVIGATING" if self.busy else "SEARCHING"
        if state != self.published_state:
            self.published_state = state
            self.state_pub.publish(String(data=state))
            self.get_logger().info(f"Explorer state {state}")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self) -> None:
        now = self._now_s()
        elapsed = 0.0 if self.last_tick_s is None else max(0.0, now - self.last_tick_s)
        self.last_tick_s = now
        active = self.enabled and not self.failed and not self.stuck
        if active and self.sortie.phase == "EXPLORING":
            self.sortie.spend(elapsed)
        self._publish_state()
        if not active or self.busy or self.map is None or not self.nav.server_is_ready():
            return

        if self.sortie.phase == "EXPLORING" and self.sortie.budget_spent():
            self.sortie.start_return("sortie budget spent; keeping the reserve to get home")
        if self.sortie.phase == "RETURNING":
            self._send_goal(float(self.get_parameter("home_x").value),
                            float(self.get_parameter("home_y").value), "home")
            self._publish_state()
            return
        if self.sortie.phase != "EXPLORING":
            return

        robot_x, robot_y = self.map_pose()
        info = self.map.info
        candidate = choose_frontier(
            self.map.data,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            robot_x,
            robot_y,
            int(self.get_parameter("min_cluster_cells").value),
            blacklist=self.blacklist,
            visited=self.visited,
            min_x=float(self.get_parameter("entrance_x").value),
        )
        if candidate is None:
            if self.sortie.no_frontier():
                self.get_logger().info(f"Exploration complete: {self.sortie.reason}; returning to the entrance")
                self._publish_state()
            return
        self.sortie.frontier_found()
        _score, x, y, cell_count = candidate
        self.get_logger().info(f"Frontier selected x={x:.2f} y={y:.2f} cells={cell_count}")
        self._send_goal(x, y, "frontier")

    def _send_goal(self, x: float, y: float, kind: str) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self.busy = True
        self.nav.send_goal_async(goal).add_done_callback(
            lambda future: self._goal_sent(future, x, y, kind)
        )

    def _goal_sent(self, future, x: float, y: float, kind: str) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.busy = False
            self._goal_failed(x, y, kind)
            self.get_logger().warning(f"{kind} goal send failed: {exc}")
            return

        self.goal_handle = handle
        if not handle.accepted:
            self.busy = False
            self._goal_failed(x, y, kind)
            return
        if self.failed or self.stuck or not self.enabled:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda result_future: self._done(result_future, x, y, kind)
        )

    def _goal_failed(self, x: float, y: float, kind: str) -> None:
        if kind == "home":
            if self.sortie.return_failed():
                self.get_logger().error("Return to the entrance failed repeatedly; Writer cannot get home")
        elif not self.failed and not self.stuck and self.enabled:
            self.blacklist.append((x, y))

    def _done(self, future, x: float, y: float, kind: str) -> None:
        self.busy = False
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception:
            status = -1
        if status == 4:
            if kind == "home":
                self.sortie.arrived_home()
                self.get_logger().info("Writer is back at the entrance with the full map")
            else:
                self.visited.append((x, y))
        elif not self.failed and not self.stuck and self.enabled:
            self._goal_failed(x, y, kind)
        self._publish_state()


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
