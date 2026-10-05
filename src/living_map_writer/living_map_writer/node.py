from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from living_map_interfaces.msg import RobotStatus

from .battery import Battery
from .progress import StuckDetector

DRIVING_STATES = {"NAVIGATING", "RETURNING"}


class Writer(Node):
    """Writer supervisor: lifecycle, faults and on-board telemetry.

    Telemetry goes to the Writer's own radio (``inside/writer_status``); the
    radio mesh decides whether it reaches the gateway. Two failure modes are
    modelled: DESTROYED (radio dies after a short dying gasp) and STUCK (the
    robot is immobilised but alive, so its radio keeps reporting and relaying).
    """

    def __init__(self) -> None:
        super().__init__("writer", namespace="/living_map")
        self.failed = False
        self.stuck = False
        self.stuck_reason = ""
        self.explorer_state = ""
        self.declare_parameter("auto_explore", False)
        self.declare_parameter("stuck_timeout_s", 45.0)
        self.declare_parameter("stuck_min_progress_m", 0.3)
        self.declare_parameter("dying_gasp_s", 3.0)
        self.declare_parameter("battery_start_pct", 100.0)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.battery = Battery(level_pct=float(self.get_parameter("battery_start_pct").value))
        self.failure_reason = ""
        self.return_requested = False
        self.last_tick_s: float | None = None
        self.enabled = bool(self.get_parameter("auto_explore").value)
        # The battery drains only once the sortie has started: a Writer waiting at the
        # entrance for the operator is on standby, not on the clock.
        self.sortie_started = self.enabled
        self.detector = StuckDetector(
            float(self.get_parameter("stuck_timeout_s").value),
            float(self.get_parameter("stuck_min_progress_m").value),
        )
        self.failed_at: float | None = None
        self.last_odom_pose: Pose | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd = self.create_publisher(Twist, "/writer/cmd_vel", 10)
        self.status = self.create_publisher(RobotStatus, "inside/writer_status", 10)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.failed_pub = self.create_publisher(Bool, "writer_failed", qos)
        self.enabled_pub = self.create_publisher(Bool, "writer_enabled", qos)
        self.stuck_pub = self.create_publisher(Bool, "writer_stuck", qos)
        self.return_pub = self.create_publisher(Bool, "writer_return_request", qos)
        self.create_service(SetBool, "writer/set_enabled", self.set_enabled)
        self.create_service(Trigger, "writer/fail", self.fail)
        self.create_service(SetBool, "writer/set_stuck", self.set_stuck)
        self.create_subscription(Odometry, "/writer/odom", self.on_odom, 10)
        self.create_subscription(String, "writer_explorer_state", self.on_explorer_state, qos)
        self.create_timer(0.5, self.tick)
        self.failed_pub.publish(Bool(data=False))
        self.enabled_pub.publish(Bool(data=self.enabled))
        self.stuck_pub.publish(Bool(data=False))
        self.return_pub.publish(Bool(data=False))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def set_enabled(self, request, response):
        if self.failed:
            response.success = False
            response.message = "Writer has failed; restart the demo to explore again."
            return response
        if self.stuck and request.data:
            response.success = False
            response.message = "Writer is immobilised; brief the Executor instead."
            return response
        if self.explorer_state == "HOME" and request.data:
            response.success = False
            response.message = "Writer has completed its sortie and is back at the entrance."
            return response
        self.enabled = bool(request.data)
        self.sortie_started = self.sortie_started or self.enabled
        self.enabled_pub.publish(Bool(data=self.enabled))
        if not self.enabled:
            self.cmd.publish(Twist())
        response.success = True
        response.message = "Writer exploring" if self.enabled else "Writer paused"
        return response

    def on_odom(self, msg: Odometry) -> None:
        self.last_odom_pose = msg.pose.pose

    def on_explorer_state(self, msg: String) -> None:
        self.explorer_state = msg.data
        if msg.data == "RETURN_FAILED" and not self.stuck:
            self._become_stuck("could not drive back to the entrance")

    def _go_down(self, reason: str) -> None:
        self.failed = True
        self.failure_reason = reason
        self.failed_at = self._now_s()
        self.cmd.publish(Twist())
        self.failed_pub.publish(Bool(data=True))
        self.get_logger().error(f"WRITER LOST ({reason}); radio sends a dying gasp, then goes silent")

    def fail(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._go_down("destroyed (fault injection)")
        response.success = True
        response.message = "Writer destroyed (fault injection); persistent beacons remain independent."
        return response

    def _become_stuck(self, reason: str) -> None:
        self.stuck = True
        self.stuck_reason = reason
        self.cmd.publish(Twist())
        self.stuck_pub.publish(Bool(data=True))
        self.get_logger().error(f"WRITER STUCK: {reason}; radio stays alive as a relay")

    def set_stuck(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if self.failed:
            response.success = False
            response.message = "Writer is destroyed."
            return response
        if request.data:
            self._become_stuck("immobilised (fault injection)")
            response.message = "Writer immobilised (fault injection); it keeps reporting and relaying."
        else:
            self.stuck = False
            self.stuck_reason = ""
            self.detector.reset()
            self.stuck_pub.publish(Bool(data=False))
            response.message = "Writer freed"
        response.success = True
        return response

    def map_pose(self) -> Pose | None:
        try:
            transform = self.tf_buffer.lookup_transform("map", "writer/base_link", Time())
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except TransformException:
            return self.last_odom_pose

    def _state(self) -> tuple[str, str]:
        if self.failed:
            return "FAILED", f"Lost: {self.failure_reason}; last transmission before radio loss"
        if self.stuck:
            return "STUCK", f"Stuck: {self.stuck_reason}; still reporting and relaying"
        if self.explorer_state == "HOME":
            return "HOME", "Back at the entrance: sortie complete, full map uploaded"
        if self.explorer_state == "RETURNING":
            why = "battery reserve reached" if self.return_requested else "exploration finished"
            return "RETURNING", f"{why[0].upper()}{why[1:]}; returning to the entrance"
        if self.enabled:
            return "ONLINE", "Exploring"
        return "PAUSED", "Ready to explore"

    def _update_battery(self, now: float, pose: Pose | None) -> None:
        dt = 0.0 if self.last_tick_s is None else max(0.0, now - self.last_tick_s)
        self.last_tick_s = now
        if self.failed or not self.sortie_started:
            return
        point = None if pose is None else (pose.position.x, pose.position.y)
        self.battery.update(dt, point)
        if self.battery.depleted:
            self._go_down("battery depleted")
            return
        home = (float(self.get_parameter("home_x").value), float(self.get_parameter("home_y").value))
        exploring = self.enabled and self.explorer_state in {"SEARCHING", "NAVIGATING"}
        if exploring and not self.return_requested and point is not None and self.battery.must_return(point, home):
            self.return_requested = True
            self.return_pub.publish(Bool(data=True))
            self.get_logger().warning(
                f"Battery {self.battery.level:.0f}% reached the reserve needed to get home; turning back")

    def tick(self) -> None:
        now = self._now_s()
        if self.failed or self.stuck or not self.enabled:
            self.cmd.publish(Twist())
        if self.failed and self.failed_at is not None and now - self.failed_at > float(
                self.get_parameter("dying_gasp_s").value):
            return  # The radio died with the robot.

        pose = self.map_pose()
        self._update_battery(now, pose)
        if pose is not None and not self.failed and not self.stuck:
            driving = self.enabled and self.explorer_state in DRIVING_STATES
            if self.detector.update(now, pose.position.x, pose.position.y, driving):
                self._become_stuck(
                    f"no progress for {self.detector.window_s:.0f} s while "
                    f"{self.explorer_state.lower()}"
                )

        msg = RobotStatus()
        msg.robot_id = "writer"
        msg.state, msg.detail = self._state()
        msg.heartbeat = self.get_clock().now().to_msg()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = msg.heartbeat
        if pose is not None:
            msg.pose.pose = pose
        msg.battery_percent = float(self.battery.level)
        self.status.publish(msg)


def main() -> None:
    rclpy.init()
    node = Writer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
