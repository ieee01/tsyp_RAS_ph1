"""Writer node wiring of stuck detection (unit level, needs rclpy to import).

These call the real ``Writer`` methods on a stand-in object. They do not
start a ROS executor, create subscriptions, exercise DDS/QoS or TF, or run
Nav2 or Gazebo: they check which data the node passes to the detector.
"""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("living_map_interfaces.msg")
from geometry_msgs.msg import Pose, Twist  # noqa: E402
from rclpy.time import Time  # noqa: E402

from living_map_writer.node import Writer  # noqa: E402
from living_map_writer.progress import TRANSLATE, UNCONFIRMED, UNKNOWN, ZERO, StuckDetector  # noqa: E402


def pose_at(x, y, yaw):
    pose = Pose()
    pose.position.x, pose.position.y = x, y
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


def twist(linear, angular):
    msg = Twist()
    msg.linear.x, msg.angular.z = linear, angular
    return msg


def odom(x, y, yaw):
    return SimpleNamespace(pose=SimpleNamespace(pose=pose_at(x, y, yaw)))


def writer(window_s=5.0, enabled=False, state="PAUSED", map_motion=lambda t: (0.0, 0.0, 0.0)):
    w = SimpleNamespace(
        failed=False, stuck=False, enabled=enabled, explorer_state=state, failed_at=None,
        sortie_started=enabled, odom_stale_reported=False, last_odom_pose=None,
        cmd=Mock(), status=Mock(), enabled_pub=Mock(), stuck_pub=Mock(),
        battery=SimpleNamespace(level=90.0), detector=StuckDetector(window_s=window_s),
        clock_s=0.0, logger=Mock(), _update_battery=Mock(), _become_stuck=Mock(),
        _state=lambda: ("ONLINE", "Exploring"),
        get_parameter=lambda name: SimpleNamespace(value={"dying_gasp_s": 3.0}[name]),
        get_clock=lambda: SimpleNamespace(now=lambda: Time()),
    )
    w._become_stuck.side_effect = lambda reason: setattr(w, "stuck", True)  # latches, as the node does
    w.get_logger = lambda: w.logger
    w._now_s = lambda: w.clock_s
    w.map_pose = lambda: pose_at(*map_motion(w.clock_s))
    w._detector_active = lambda: Writer._detector_active(w)
    w._sync_detector = lambda: Writer._sync_detector(w)
    return w


def deliver_own_commands(w):
    """The Writer hears its own publications on /writer/cmd_vel."""
    for call in w.cmd.publish.call_args_list:
        Writer.on_cmd_vel(w, call.args[0])
    w.cmd.publish.reset_mock()


def start_navigation(w):
    w.enabled = True
    Writer.on_explorer_state(w, SimpleNamespace(data="NAVIGATING"))


def run(w, seconds, command=None, motion=lambda t: (0.0, 0.0, 0.0), odom_alive=True):
    """Advance 100 Hz: commands at ~12 Hz, odometry at ~33 Hz, tick at 2 Hz."""
    start = round(w.clock_s * 100)
    for k in range(start + 1, start + round(seconds * 100) + 1):
        w.clock_s = k / 100
        if command is not None and k % 8 == 0:
            Writer.on_cmd_vel(w, twist(*command))
        if odom_alive and k % 3 == 0:
            Writer.on_odom(w, odom(*motion(w.clock_s)))
        if k % 50 == 0:
            Writer.tick(w)
            deliver_own_commands(w)


def test_startup_while_paused_ignores_the_writers_own_zero_commands():
    w = writer()
    run(w, 10.0)  # paused: tick publishes zeros, echoed back to on_cmd_vel
    assert not w.detector.active
    start_navigation(w)
    assert w.detector.command_state(w.clock_s) == UNKNOWN


def test_writer_still_publishes_stops_when_paused_stuck_or_failed():
    """Safety: the stop path is the original one, untouched by stuck detection."""
    for flags in ({"enabled": False}, {"enabled": True, "stuck": True}, {"enabled": True, "failed": True}):
        w = writer(enabled=True, state="NAVIGATING")
        for name, value in flags.items():
            setattr(w, name, value)
        w.clock_s = 0.5
        Writer.tick(w)
        published = w.cmd.publish.call_args_list
        assert published, flags
        stop = published[0].args[0]
        assert (stop.linear.x, stop.angular.z) == (0.0, 0.0)


def test_delayed_own_stop_after_navigation_starts_cannot_mask_a_dead_nav2_stream():
    """Race: the paused Writer publishes a stop, navigation starts, then the stop arrives."""
    w = writer()
    w.clock_s = 0.5
    Writer.tick(w)              # paused: publishes a stop
    start_navigation(w)         # operator resumes before the stop is delivered
    deliver_own_commands(w)     # it arrives while the detector is active: recorded, not dropped
    assert w.detector.command_state(w.clock_s) == UNCONFIRMED
    run(w, 10.0, command=None)  # Nav2's commands never arrive; the robot does not move
    w._become_stuck.assert_called_once()  # position-only fallback, not masked


def test_lost_own_stops_never_cause_a_nav2_zero_to_be_discarded():
    """The counter in v2 would have dropped these zeros; nothing is dropped now."""
    w = writer()
    run(w, 5.0)                 # paused: ten stops published...
    w.cmd.publish.reset_mock()
    start_navigation(w)         # ...and their echoes never arrive
    Writer.on_cmd_vel(w, twist(0.5, 0.0))   # Nav2 drives
    Writer.on_cmd_vel(w, twist(0.0, 0.0))   # Nav2 stops
    assert w.detector.command_state(w.clock_s) == ZERO


def test_resuming_navigation_clears_the_stored_command():
    w = writer(enabled=True, state="NAVIGATING")
    run(w, 1.0, command=(0.5, 0.0), motion=lambda t: (0.5 * t, 0.0, 0.0))
    assert w.detector.command_state(w.clock_s) == TRANSLATE
    Writer.set_enabled(w, SimpleNamespace(data=False), SimpleNamespace())
    run(w, 2.0)  # paused; own zeros echoed
    Writer.set_enabled(w, SimpleNamespace(data=True), SimpleNamespace())
    assert w.detector.active and w.detector.command_state(w.clock_s) == UNKNOWN


def test_commands_between_goals_are_ignored():
    w = writer(enabled=True, state="SEARCHING")
    run(w, 1.0, command=(0.5, 0.0))
    Writer.on_explorer_state(w, SimpleNamespace(data="NAVIGATING"))
    assert w.detector.command_state(w.clock_s) == UNKNOWN


def test_motion_is_measured_from_odometry_not_the_map_pose():
    # The map pose swings around (SLAM corrections) while odometry shows no motion.
    w = writer(enabled=True, state="NAVIGATING", map_motion=lambda t: (math.sin(t), 0.0, t))
    run(w, 10.0, command=(0.5, 0.0))
    w._become_stuck.assert_called_once()
    assert "commanded motion produced no progress for 5 s" in w._become_stuck.call_args.args[0]
    # The map pose is still while odometry shows the robot driving.
    w = writer(enabled=True, state="NAVIGATING")
    run(w, 10.0, command=(0.5, 0.0), motion=lambda t: (0.5 * t, 0.0, 0.0))
    w._become_stuck.assert_not_called()


def test_obeyed_rotation_in_odometry_is_not_stuck():
    w = writer(enabled=True, state="NAVIGATING")
    run(w, 20.0, command=(0.0, 1.0), motion=lambda t: (0.0, 0.0, 1.0 * t))
    w._become_stuck.assert_not_called()


def test_stale_odometry_pauses_detection_and_warns_once():
    w = writer(enabled=True, state="NAVIGATING")
    run(w, 20.0, command=(0.5, 0.0), odom_alive=False)
    w._become_stuck.assert_not_called()
    assert w.logger.warning.call_count == 1
    assert w.odom_stale_reported


def test_no_command_stream_falls_back_to_position_only():
    w = writer(enabled=True, state="NAVIGATING")
    run(w, 10.0, command=None, motion=lambda t: (0.0, 0.0, 1.0 * t))
    w._become_stuck.assert_called_once()
