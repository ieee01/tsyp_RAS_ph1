"""Stuck detection: commanded motion versus odometry-measured motion.

These are MATHEMATICAL SIMULATIONS of the detector in isolation. No ROS,
Nav2 or Gazebo code runs here. ``simulate`` steps a planar robot at 100 Hz,
publishes commands at the rates of this repository's Nav2 configuration,
feeds odometry at ~33 Hz (Gazebo publishes 30 Hz) and evaluates every 0.5 s,
as ``Writer.tick`` does. Unless a phase forces a velocity (a blocked robot),
the robot executes the latest command it received, ramped by the Writer
DiffDrive acceleration limits in ``models/writer/model.sdf``.

The Nav2 timelines follow ``config/nav2/writer.yaml`` and the launch file:
RPP at ``controller_frequency: 12.0`` with ``rotate_to_heading_angular_vel``
1.0 rad/s; ``failure_tolerance: 0.3`` (RPP raises NoValidControl when a
collision is predicted, so FollowPath aborts about 0.3 s later); progress
checker 0.3 m in 12 s; behaviour server at 10 Hz, ``max_rotational_vel`` 1.0,
``rotational_acc_lim`` 2.0; and the Jazzy default tree (not overridden):
6 retries, round robin of clear costmaps, Spin 1.57 rad, Wait 5 s,
BackUp 0.30 m at 0.15 m/s. Real timings in Gazebo will differ; these
scenarios have not been validated in Gazebo.
"""
import math
import random

import pytest

from living_map_writer.progress import (INVALID, ROTATE, STALE, TRANSLATE, UNCONFIRMED, UNKNOWN,
                                        ZERO, StuckDetector, yaw_from_quaternion)

DT = 0.01
ODOM_EVERY = 3     # ~33 Hz odometry
TICK_EVERY = 50    # Writer.tick every 0.5 s
LINEAR_ACC, ANGULAR_ACC = 0.5, 1.5   # DiffDrive limits, Writer model.sdf
RPP_HZ, BEHAVIOR_HZ = 12.0, 10.0


def phase(duration, cmd=None, rate=RPP_HZ, act=None, label="", active=True, extra=()):
    """``cmd`` published at ``rate`` Hz (rate 0: once, at the phase start; None: silent).

    ``act``: None, the robot executes its latest command; (v, w), forced velocity.
    ``extra``: more publishers in this phase, as (cmd, rate, offset_s).
    ``active``: whether the Writer is in a navigation state (detector active).
    """
    return {"duration": duration, "cmd": cmd, "rate": rate, "act": act, "label": label,
            "active": active, "extra": tuple(extra)}


def _due(i, rate, offset):
    start = round(offset / DT)
    if i < start:
        return False
    if rate == 0:
        return i == start
    return (i - start) % max(1, round(1.0 / (rate * DT))) == 0


def _ramp(current, target, limit):
    step = limit * DT
    return current + max(-step, min(step, target - current))


def simulate(detector, phases, *, noise=None, seed=7, drop_commands=False,
             odom_alive=lambda t: True, yaw_available=True):
    """Return (first STUCK time or None, label of the phase where it happened)."""
    rng = random.Random(seed)
    x = y = yaw = v = w = 0.0
    drive_cmd = (0.0, 0.0)  # what the drive plugin executes: the latest command
    k = 0
    for item in phases:
        detector.set_active(item["active"])
        publishers = ([(item["cmd"], item["rate"], 0.0)] if item["cmd"] is not None else [])
        publishers += list(item["extra"])
        for i in range(round(item["duration"] / DT)):
            t = k * DT
            for cmd, rate, offset in publishers:
                if _due(i, rate, offset):
                    drive_cmd = cmd
                    if not drop_commands:
                        detector.command(t, *cmd)
            if item["act"] is None:
                v = _ramp(v, drive_cmd[0], LINEAR_ACC)
                w = _ramp(w, drive_cmd[1], ANGULAR_ACC)
            else:
                v, w = item["act"]
            yaw += w * DT
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            k += 1
            t = k * DT
            if k % ODOM_EVERY == 0 and odom_alive(t):
                nx, ny, nyaw = x, y, yaw
                if noise:
                    nx += rng.uniform(-noise[0], noise[0])
                    ny += rng.uniform(-noise[0], noise[0])
                    nyaw += rng.uniform(-noise[1], noise[1])
                detector.odometry(t, nx, ny, nyaw if yaw_available else None)
            if k % TICK_EVERY == 0 and detector.evaluate(t):
                return round(t, 2), item["label"]
    return None, None


IMMOBILE = (0.0, 0.0)


# -- Nav2 timelines built from this repository's configuration -------------
def rotate_to_heading(seconds=1.6):
    return [phase(seconds, (0.0, 1.0), RPP_HZ, label="RPP rotate to heading")]


def followpath_collision():
    # RPP predicts a collision, publishes zero and fails; failure_tolerance 0.3 s.
    return [phase(0.3, (0.0, 0.0), RPP_HZ, label="FollowPath: collision ahead")]


def navigate_attempt_collision():
    # ComputePathToPose, FollowPath fails, local costmap cleared, FollowPath fails again.
    return ([phase(0.2, None, label="ComputePathToPose")] + followpath_collision()
            + [phase(0.05, None, label="clear local costmap")] + followpath_collision())


def spin():
    # 1.57 rad: accelerate at 2 rad/s^2 to 1 rad/s, cruise, decelerate; then stop.
    profile = [(0.25, 0.25), (0.25, 0.75), (1.07, 1.0), (0.25, 0.75), (0.25, 0.25)]
    return ([phase(d, (0.0, rate), BEHAVIOR_HZ, label="Spin") for d, rate in profile]
            + [phase(0.1, (0.0, 0.0), 0, label="Spin stop")])


def wait():
    return [phase(5.0, None, label="Wait")]  # the Wait behaviour publishes no command


def backup():
    return [phase(2.0, (-0.15, 0.0), BEHAVIOR_HZ, label="BackUp"),
            phase(0.1, (0.0, 0.0), 0, label="BackUp stop")]


def clear_costmaps():
    return [phase(0.05, None, label="clear costmaps")]


def collision_goal():
    """One frontier goal blocked by an obstacle the costmap sees, until Nav2 aborts it."""
    recoveries = [clear_costmaps(), spin(), wait(), backup(), clear_costmaps(), spin()]
    timeline = rotate_to_heading() + navigate_attempt_collision()
    for recovery in recoveries:
        timeline += recovery + navigate_attempt_collision()
    return timeline


def wedged_goal(attempts=4):
    """The controller keeps commanding (obstacle below the LiDAR); the robot cannot move."""
    timeline = rotate_to_heading()
    for _ in range(attempts):
        timeline += [phase(12.0, (0.55, 0.0), RPP_HZ, label="FollowPath until progress checker fails"),
                     phase(0.05, None, label="clear local costmap")]
    return [dict(item, act=IMMOBILE) for item in timeline]


def duration(phases):
    return sum(item["duration"] for item in phases)


def immobilised(phases):
    return [dict(item, act=IMMOBILE) for item in phases]


# == Recovery timelines (this repository's Nav2 configuration) ==============
def test_collision_goal_lasts_about_twenty_seconds():
    assert 15.0 < duration(collision_goal()) < 25.0


def test_collision_recovery_of_a_mobile_robot_is_never_stuck():
    detector = StuckDetector(window_s=45)
    assert simulate(detector, collision_goal()) == (None, None)


def test_position_only_rule_has_no_false_positive_in_one_goal_at_45_s():
    """No demonstrated 45 s false positive: the goal is aborted (and detection
    restarts) long before 45 s of non-translation accumulate."""
    assert simulate(StuckDetector(window_s=45), collision_goal(), drop_commands=True) == (None, None)


def test_position_only_rule_would_fire_with_a_shorter_window_but_new_rule_does_not():
    """Why the window could not be shortened before: recoveries look like immobility.
    In this timeline the full BackUp travels ~0.30 m at t ~ 14.9 s and resets the
    position-only rule, so it fires only for windows below ~14.5 s."""
    assert simulate(StuckDetector(window_s=12), collision_goal(), drop_commands=True)[0] is not None
    assert simulate(StuckDetector(window_s=15), collision_goal(), drop_commands=True) == (None, None)
    assert simulate(StuckDetector(window_s=12), collision_goal()) == (None, None)


def test_wedged_robot_with_commanding_controller_is_stuck_after_45_s_of_commands():
    stuck_at, label = simulate(StuckDetector(window_s=45), wedged_goal())
    assert label == "FollowPath until progress checker fails"
    assert 45.0 <= stuck_at <= 46.5


def test_wedged_robot_whose_controller_refuses_to_drive_collects_only_commanded_time():
    """Residual limitation: zero commands are neutral, so only the rotate,
    Spin and BackUp seconds count within one goal. RETURN_FAILED remains the backstop."""
    detector = StuckDetector(window_s=45)
    assert simulate(detector, immobilised(collision_goal())) == (None, None)
    assert 5.0 < detector.evidence_s < 10.0


# == Command periods: never received / fresh zero / stale ===================
def test_startup_while_paused_ignores_commands_until_navigation_starts():
    detector = StuckDetector(window_s=10)
    for i in range(20):  # Writer paused: its own zeros arrive at 2 Hz
        detector.command(i * 0.5, 0.0, 0.0)
    assert detector.command_state(10.0) == UNKNOWN
    detector.set_active(True)
    assert detector.command_state(10.0) == UNKNOWN


def test_resuming_navigation_forgets_the_previous_command():
    detector = StuckDetector(window_s=10)
    detector.set_active(True)
    detector.command(1.0, 0.5, 0.0)
    assert detector.command_state(1.2) == TRANSLATE
    detector.set_active(False)      # pause
    detector.command(1.5, 0.0, 0.0)  # Writer's own stop: ignored
    detector.set_active(True)       # resume
    assert detector.command_state(1.6) == UNKNOWN


def test_never_received_unconfirmed_fresh_zero_and_stale_are_distinguished():
    detector = StuckDetector(window_s=10, cmd_timeout_s=1.0)
    detector.set_active(True)
    assert detector.command_state(0.0) == UNKNOWN
    detector.command(0.5, 0.0, 0.0)               # a zero from anyone: proves nothing yet
    assert detector.command_state(0.6) == UNCONFIRMED
    assert detector.command_state(5.0) == UNCONFIRMED  # going stale does not make it neutral
    detector.command(1.0, 0.0, 0.8)               # first non-zero command confirms the stream
    assert detector.command_state(1.1) == ROTATE
    detector.command(1.2, 0.0, 0.0)
    assert detector.command_state(1.5) == ZERO
    assert detector.command_state(2.5) == STALE
    detector.command(3.0, 0.5, 0.0)
    assert detector.command_state(3.1) == TRANSLATE
    detector.command(3.2, math.nan, 0.0)
    assert detector.command_state(3.3) == INVALID


def test_invalid_command_does_not_confirm_the_stream():
    detector = StuckDetector(window_s=10)
    detector.set_active(True)
    detector.command(0.0, math.nan, 1.0)
    detector.command(0.1, 0.0, 0.0)
    assert detector.command_state(0.2) == UNCONFIRMED


def test_missing_commands_for_a_whole_period_fall_back_to_position_only():
    detector = StuckDetector(window_s=10)
    stuck_at, _ = simulate(detector, [phase(20, (0.0, 1.0))], drop_commands=True)
    # Rotation cannot be confirmed as commanded; evidence starts at the first
    # odometry sample (t = 0.03 s), so it reaches 10 s just after the 10.0 s tick.
    assert stuck_at == pytest.approx(10.0, abs=0.51)
    detector = StuckDetector(window_s=10)
    assert simulate(detector, [phase(30, (0.5, 0.0))], drop_commands=True) == (None, None)


def test_fresh_zero_and_stale_commands_hold_evidence():
    detector = StuckDetector(window_s=45)
    phases = [phase(30, (0.5, 0.0), act=IMMOBILE, label="push"),
              phase(30, (0.0, 0.0), RPP_HZ, act=IMMOBILE, label="fresh zero"),
              phase(30, (0.0, 0.0), 0, act=IMMOBILE, label="stale"),
              phase(30, (0.5, 0.0), act=IMMOBILE, label="push again")]
    stuck_at, label = simulate(detector, phases)
    assert label == "push again" and stuck_at == pytest.approx(105.0, abs=0.51)


def test_invalid_commands_count_as_evidence():
    detector = StuckDetector(window_s=10)
    phases = [phase(20, (math.nan, 0.0), act=(0.0, 1.0))]
    assert simulate(detector, phases)[0] == pytest.approx(10.0, abs=0.51)


def test_stale_command_inside_a_step_is_split_at_the_timeout():
    detector = StuckDetector(window_s=10, cmd_timeout_s=1.0)
    detector.set_active(True)
    detector.odometry(0.0, 0.0, 0.0, 0.0)
    detector.command(0.0, 0.5, 0.0)
    detector.odometry(0.0 + 0.9, 0.0, 0.0, 0.0)  # robot does not move
    detector.odometry(1.6, 0.0, 0.0, 0.0)        # command stale from t = 1.0
    detector.evaluate(1.6)
    assert detector.evidence_s == pytest.approx(1.0)


# == Odometry ===============================================================
def test_stale_odometry_neither_accuses_nor_clears():
    detector = StuckDetector(window_s=10)
    phases = [phase(5, (0.5, 0.0), act=IMMOBILE), phase(30, (0.5, 0.0), act=IMMOBILE)]
    stuck_at, _ = simulate(detector, phases, odom_alive=lambda t: t < 5.0)
    assert stuck_at is None
    assert detector.odom_stale(35.0)
    assert 4.0 < detector.evidence_s <= 5.0


def test_odometry_gap_is_not_attributed():
    detector = StuckDetector(window_s=10, max_step_s=1.0)
    detector.set_active(True)
    detector.odometry(0.0, 0.0, 0.0, 0.0)
    detector.command(0.0, 0.5, 0.0)
    detector.odometry(30.0, 0.0, 0.0, 0.0)  # 30 s without odometry
    detector.evaluate(30.0)
    assert detector.evidence_s == 0.0


def test_odometry_resuming_after_a_gap_counts_again():
    detector = StuckDetector(window_s=10)
    phases = [phase(40, (0.5, 0.0), act=IMMOBILE)]
    stuck_at, _ = simulate(detector, phases, odom_alive=lambda t: not 5.0 < t < 25.0)
    assert stuck_at == pytest.approx(5.0 + 20.0 + 5.0, abs=0.6)


def test_invalid_orientation_makes_rotation_unverifiable():
    detector = StuckDetector(window_s=10)
    assert simulate(detector, [phase(20, (0.0, 1.0))], yaw_available=False)[0] == pytest.approx(10.0, abs=0.51)


def test_odometry_jump_is_not_progress():
    detector = StuckDetector(window_s=10)
    detector.set_active(True)
    for i in range(10):
        detector.odometry(i * 0.5, 0.0, 0.0, 0.0)
        detector.command(i * 0.5, 0.5, 0.0)
        detector.evaluate(i * 0.5)
    detector.odometry(5.0, 3.0, 0.0, 0.0)  # 3 m in 0.5 s
    detector.evaluate(5.0)
    assert detector.evidence_s == pytest.approx(4.5)


def test_clock_going_backwards_restarts_the_period():
    detector = StuckDetector(window_s=10)
    detector.set_active(True)
    for i in range(10):
        detector.update(100 + i * 0.5, 0.0, 0.0, True, yaw=0.0, cmd=(0.5, 0.0))
    detector.odometry(1.0, 0.0, 0.0, 0.0)
    assert detector.evidence_s == 0.0 and detector.command_state(1.0) == UNKNOWN


def test_non_finite_pose_samples_are_ignored():
    detector = StuckDetector(window_s=10)
    detector.update(0.0, 0.0, 0.0, True, yaw=0.0, cmd=(0.5, 0.0))
    assert not detector.update(0.5, math.nan, 0.0, True, yaw=0.0, cmd=(0.5, 0.0))
    assert detector.evidence_s == 0.0


# == Command sampling within one 0.5 s tick =================================
def test_one_zero_sample_at_the_tick_does_not_hide_a_driving_command():
    """The previous patch judged a whole tick by one sample."""
    detector = StuckDetector(window_s=45)
    phases = []
    for _ in range(120):  # a stray zero (e.g. another node) just before each tick
        phases += [phase(0.45, (0.5, 0.0), act=IMMOBILE, label="push"),
                   phase(0.05, (0.0, 0.0), 0, act=IMMOBILE, label="stray zero")]
    stuck_at, _ = simulate(detector, phases)
    # 0.05 s of every 0.5 s is a fresh zero: 45 s of evidence takes 50 s.
    assert stuck_at == pytest.approx(50.0, abs=0.6)


def test_two_publishers_interleaving_are_weighted_by_time_in_force():
    """Rotation (obeyed) and translation (blocked) alternate within each tick."""
    detector = StuckDetector(window_s=10)
    tick = phase(0.5, None, act=(0.0, 1.0), label="alternating",
                 extra=[((0.0, 1.0), 0, 0.0), ((0.5, 0.0), 0, 0.25)])
    stuck_at, _ = simulate(detector, [tick] * 60)
    # Only the translating half of each tick is evidence: 10 s takes ~20 s.
    assert stuck_at == pytest.approx(20.0, abs=0.6)


def test_rotation_then_translation_within_a_tick():
    detector = StuckDetector(window_s=10)
    detector.set_active(True)
    detector.odometry(0.0, 0.0, 0.0, 0.0)
    detector.command(0.0, 0.0, 1.0)
    detector.odometry(0.25, 0.0, 0.0, 0.25)    # turned as commanded
    detector.command(0.25, 0.5, 0.0)
    detector.odometry(0.5, 0.0, 0.0, 0.25)     # told to drive, did not move
    detector.evaluate(0.5)
    assert detector.evidence_s == pytest.approx(0.25)


def test_zero_commands_after_nav2_has_commanded_motion_are_neutral():
    """demo_fallback stops at each waypoint; Nav2 publishes zero when it stops."""
    detector = StuckDetector(window_s=10)
    phases = [phase(2, (0.5, 0.0)), phase(60, (0.0, 0.0), 5.0, act=IMMOBILE)]
    assert simulate(detector, phases) == (None, None)


# == Zero commands from several publishers: delayed, missing, reordered ======
WRITER_STOP_RATE = 2.0  # Writer.tick publishes a stop every 0.5 s while paused/stuck/failed


def test_only_zero_commands_never_mask_a_dead_nav2_stream():
    """Nav2's commands never arrive; zeros from the Writer and others do."""
    detector = StuckDetector(window_s=10)
    phases = [phase(30, (0.0, 0.0), WRITER_STOP_RATE, act=IMMOBILE,
                    extra=[((0.0, 0.0), 5.0, 0.13)], label="zeros only")]
    stuck_at, _ = simulate(detector, phases)
    assert stuck_at == pytest.approx(10.0, abs=0.51)  # same as the position-only rule


def test_delayed_writer_stop_after_navigation_starts_is_not_nav2s_zero():
    detector = StuckDetector(window_s=10)
    phases = [phase(0.4, (0.0, 0.0), 0, act=IMMOBILE, label="late echo of the Writer's stop"),
              phase(12, None, act=IMMOBILE, label="Nav2 silent")]
    stuck_at, _ = simulate(detector, phases)
    assert stuck_at == pytest.approx(10.0, abs=0.51)
    assert detector.command_state(12.0) == UNCONFIRMED


def test_missing_writer_echo_does_not_cause_nav2_zeros_to_be_dropped():
    """No message is discarded: a real Nav2 zero is recorded as a fresh zero."""
    detector = StuckDetector(window_s=10)
    phases = [phase(1.0, (0.5, 0.0), label="Nav2 drives"),
              phase(20, (0.0, 0.0), RPP_HZ, act=IMMOBILE, label="Nav2 holds zero")]
    assert simulate(detector, phases) == (None, None)
    assert detector.command_state(21.0) == ZERO


def test_writer_echo_reordered_after_nav2_commands_costs_at_most_one_command_period():
    """An old stop arriving after Nav2's first command is in force only until Nav2's next one."""
    pushing = [phase(60, (0.5, 0.0), RPP_HZ, act=IMMOBILE, label="push")]
    with_echo = [dict(pushing[0], extra=[((0.0, 0.0), 0, 0.04)])]
    clean, _ = simulate(StuckDetector(window_s=45), pushing)
    late, _ = simulate(StuckDetector(window_s=45), with_echo)
    assert clean == pytest.approx(45.0, abs=0.51)
    assert late - clean <= 0.5  # at most ~0.05 s of evidence lost: same or next tick


def test_interleaved_zeros_from_another_publisher_are_weighted_by_time_in_force():
    """Nav2 drives at 12 Hz; another node publishes zero twice a second."""
    detector = StuckDetector(window_s=45)
    phases = [phase(70, (0.5, 0.0), RPP_HZ, act=IMMOBILE, label="push",
                    extra=[((0.0, 0.0), 2.0, 0.04)])]
    stuck_at, _ = simulate(detector, phases)
    # Nav2 publishes every 0.08 s here; each stray zero holds until Nav2's next
    # command: 0.02-0.08 s, 0.05 s on average per 0.5 s, i.e. 10 % of the time.
    # 45 s of evidence therefore takes about 50 s.
    assert stuck_at == pytest.approx(50.0, abs=0.6)


def test_confirmed_stream_that_goes_silent_is_neutral():
    """Documented residual: after Nav2 has commanded, silence reads as stale."""
    detector = StuckDetector(window_s=10)
    phases = [phase(1.0, (0.5, 0.0), act=IMMOBILE), phase(30, None, act=IMMOBILE)]
    assert simulate(detector, phases) == (None, None)
    assert detector.command_state(31.0) == STALE


def test_nav2_zeros_before_its_first_motion_count_like_the_original_rule():
    """A goal that starts blocked: zeros until the first Spin count as evidence."""
    blocked_start = collision_goal()[1:]  # no rotate-to-heading: Nav2 starts with zeros
    detector = StuckDetector(window_s=45)
    assert simulate(detector, blocked_start) == (None, None)
    original = StuckDetector(window_s=45)
    assert simulate(original, blocked_start, drop_commands=True) == (None, None)
    assert detector.evidence_s <= original.evidence_s + 0.01


# == Rotation versus translation (the original ten scenarios) ===============
def test_forward_driving_is_never_stuck():
    detector = StuckDetector(window_s=10)
    assert simulate(detector, [phase(120, (0.55, 0.0)), phase(60, (0.4, 0.3))]) == (None, None)


def test_reversing_is_progress():
    detector = StuckDetector(window_s=10)
    assert simulate(detector, [phase(60, (-0.15, 0.0)), phase(30, (-0.3, -0.2))]) == (None, None)


def test_obeyed_rotation_in_place_is_never_stuck_however_long():
    detector = StuckDetector(window_s=10)
    assert simulate(detector, [phase(120, (0.0, 1.0)), phase(120, (0.0, -0.4))]) == (None, None)
    assert detector.evidence_s < 0.5  # at most the reversal's ramp


def test_immobilised_robot_commanded_forward_is_stuck_after_one_window():
    detector = StuckDetector(window_s=45)
    stuck_at, _ = simulate(detector, [phase(10, (0.5, 0.0)), phase(60, (0.5, 0.0), act=IMMOBILE)])
    assert 45.0 <= stuck_at <= 55.5


def test_yawing_while_commanded_to_drive_is_still_stuck():
    detector = StuckDetector(window_s=45)
    stuck_at, _ = simulate(detector, [phase(60, (0.5, 0.0), act=(0.0, 0.8))])
    assert stuck_at == pytest.approx(45.0, abs=0.51)


def test_commanded_rotation_that_does_not_happen_is_stuck():
    detector = StuckDetector(window_s=45)
    assert simulate(detector, [phase(60, (0.0, 1.0), act=IMMOBILE)])[0] == pytest.approx(45.0, abs=0.51)


def test_rotation_that_lags_far_behind_the_command_is_stuck():
    detector = StuckDetector(window_s=20)
    assert simulate(detector, [phase(30, (0.0, 1.0), act=(0.0, 0.1))])[0] == pytest.approx(20.0, abs=0.51)


def test_rotation_in_the_wrong_direction_is_stuck():
    detector = StuckDetector(window_s=20)
    assert simulate(detector, [phase(30, (0.0, 1.0), act=(0.0, -1.0))])[0] == pytest.approx(20.0, abs=0.51)


def test_robot_that_can_turn_but_never_translate_is_still_stuck():
    detector = StuckDetector(window_s=45)
    cycle = [phase(10, (0.4, 0.0), act=IMMOBILE, label="push"), phase(3, (0.0, 1.0), label="turn")]
    stuck_at, label = simulate(detector, cycle * 6)
    assert label == "push"
    assert stuck_at == pytest.approx(45.0 + 4 * 3.0, abs=1.1)


def test_pause_restarts_detection():
    detector = StuckDetector(window_s=45)
    phases = [phase(40, (0.5, 0.0), act=IMMOBILE), phase(5, None, act=IMMOBILE, active=False),
              phase(40, (0.5, 0.0), act=IMMOBILE)]
    assert simulate(detector, phases) == (None, None)


def test_position_and_heading_noise_do_not_fake_progress():
    for cmd in ((0.5, 0.0), (0.0, 1.0)):
        detector = StuckDetector(window_s=45)
        stuck_at, _ = simulate(detector, [phase(60, cmd, act=IMMOBILE)], noise=(0.02, 0.015))
        assert stuck_at == pytest.approx(45.0, abs=0.51)


def test_noise_on_an_idle_or_rotating_robot_is_not_stuck():
    detector = StuckDetector(window_s=10)
    phases = [phase(60, (0.0, 0.8)), phase(60, (0.0, 0.0), 5.0)]
    assert simulate(detector, phases, noise=(0.02, 0.015)) == (None, None)


def test_slow_creep_is_progress():
    detector = StuckDetector(window_s=45)
    assert simulate(detector, [phase(200, (0.03, 0.0), act=(0.01, 0.0))]) == (None, None)


def test_full_backup_counts_as_progress():
    detector = StuckDetector(window_s=45)
    phases = [phase(30, (0.5, 0.0), act=IMMOBILE), phase(2.0, (-0.15, 0.0), act=(-0.155, 0.0))]
    simulate(detector, phases)
    assert detector.evidence_s < 1.0


# == Window boundary and single-sample interface ============================
def test_window_boundary_is_inclusive_and_exact():
    detector = StuckDetector(window_s=10)
    results = [detector.update(i * 0.5, 0.0, 0.0, True, yaw=0.0, cmd=(0.5, 0.0)) for i in range(21)]
    assert not any(results[:20]) and results[20]


def test_progress_boundary_is_inclusive():
    detector = StuckDetector(window_s=10, min_progress_m=0.25)
    for i in range(19):
        detector.update(i * 0.5, 0.0, 0.0, True, yaw=0.0, cmd=(0.5, 0.0))
    assert not detector.update(9.5, 0.25, 0.0, True, yaw=0.0, cmd=(0.5, 0.0))
    assert detector.evidence_s == 0.0


def test_disabled_window_never_fires():
    detector = StuckDetector(window_s=0)
    assert simulate(detector, [phase(60, (0.5, 0.0), act=IMMOBILE)]) == (None, None)


@pytest.mark.parametrize("cmd", [None, (math.nan, 0.0), (0.0, math.inf), ("fast", 1.0), (1.0,)])
def test_unknown_or_invalid_command_via_update_is_position_only(cmd):
    detector = StuckDetector(window_s=10)
    results = [detector.update(i * 0.5, 0.0, 0.0, True, yaw=i * 0.5, cmd=cmd) for i in range(21)]
    assert not any(results[:20]) and results[20]


def test_yaw_from_quaternion():
    half = math.radians(60) / 2
    assert yaw_from_quaternion(0, 0, math.sin(half), math.cos(half)) == pytest.approx(math.radians(60))
    assert yaw_from_quaternion(0, 0, 2 * math.sin(half), 2 * math.cos(half)) == pytest.approx(math.radians(60))
    assert yaw_from_quaternion(0, 0, 0, 0) is None
    assert yaw_from_quaternion(0, 0, math.nan, 1) is None
