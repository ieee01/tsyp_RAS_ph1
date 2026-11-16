"""Detect a Writer that is commanded to move but is physically immobilised.

The detector weighs what the robot was *told* to do (every velocity command
seen on ``/writer/cmd_vel``) against what it *did* (the ``/writer/odom``
pose). It is fed by the node's callbacks:

* ``command(stamp, linear, angular)`` for each drive command;
* ``odometry(stamp, x, y, yaw)`` for each odometry message;
* ``evaluate(now)`` once per supervisor tick (0.5 s), which turns the motion
  seen since the previous tick into evidence and returns True once STUCK.

Commands are treated like the drive plugin treats them: the latest command
received stays in force until the next one, or until it is older than
``cmd_timeout_s`` (stale). Each odometry step (about 33 ms at 30 Hz) is split
at every command change and at the moment the command goes stale, so several
publishers and command changes inside one 0.5 s tick are accounted for by
the time each command was actually in force.

What each kind of time contributes:

* **Translation commanded** (forward or reverse): evidence, until the robot
  leaves a ``min_progress_m`` disc around its last progress point. Leaving
  the disc is the only proof of mobility and clears all evidence. Yawing
  while told to drive is still evidence.
* **Rotation in place commanded**: neutral if, over the tick, the odometry
  heading followed the command (same direction, at least
  ``min_rotation_ratio`` of the commanded angle); evidence otherwise. Obeyed
  rotation never clears evidence, so a robot that can turn but not translate
  is still caught.
* **A fresh zero command, or a stale command**: neutral; evidence is held.
  This applies only once the period's command stream is *confirmed* (below).
* **No command yet in this navigation period, only zero commands so far,
  or an invalid command**: evidence, as in the original position-only rule,
  so missing command data never makes the detector less sensitive than it was.
* **No fresh odometry**: nothing is measured, so neither evidence nor
  progress accrues. ``odom_stale(now)`` reports it.

A navigation period starts when ``set_active(True)`` follows inactivity; the
stored command is cleared then. ``/writer/cmd_vel`` carries plain Twist
messages from several publishers (Nav2, the demo fallback, and the Writer's
own stop commands), which do not identify their sender, and a zero from the
Writer cannot be told from a zero from Nav2. No message is therefore
discarded or attributed. Instead, a period's command stream is confirmed only
by a non-zero command, which the Writer never publishes: zeros alone, from
whichever publisher and however delayed or reordered, can never make a dead
or unreachable Nav2 stream look like "nothing commanded".
"""
from __future__ import annotations

import math

_EPS = 1e-9

UNKNOWN = "unknown"          # no command received in this navigation period
UNCONFIRMED = "unconfirmed"  # only zero commands so far: possibly the Writer's own stops
INVALID = "invalid"          # a command with non-finite values
STALE = "stale"              # the last command is older than cmd_timeout_s
ZERO = "zero"                # a fresh command below both thresholds
TRANSLATE = "translate"      # a fresh command with |linear| >= linear_cmd_eps
ROTATE = "rotate"            # a fresh command turning in place

_BUCKETS = ("unknown", "unconfirmed", "invalid", "idle", "translate", "rotate",
            "rotate_unverified", "rot_expected", "rot_achieved")


def _finite(*values) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float | None:
    """Planar heading of a quaternion, or None if it is not a usable rotation."""
    if not _finite(x, y, z, w):
        return None
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class StuckDetector:
    def __init__(
        self,
        window_s: float = 45.0,
        min_progress_m: float = 0.3,
        *,
        linear_cmd_eps: float = 0.02,
        angular_cmd_eps: float = 0.1,
        min_rotation_ratio: float = 0.25,
        yaw_noise_rad: float = 0.02,
        cmd_timeout_s: float = 1.0,
        odom_timeout_s: float = 1.0,
        max_step_s: float = 1.0,
        max_speed_mps: float = 1.5,
    ) -> None:
        self.window_s = float(window_s)
        self.min_progress_m = float(min_progress_m)
        self.linear_cmd_eps = float(linear_cmd_eps)
        self.angular_cmd_eps = float(angular_cmd_eps)
        self.min_rotation_ratio = float(min_rotation_ratio)
        self.yaw_noise_rad = float(yaw_noise_rad)
        self.cmd_timeout_s = float(cmd_timeout_s)
        self.odom_timeout_s = float(odom_timeout_s)
        # Longest odometry gap still attributed to the commands in force; with
        # this robot's 2 rad/s limit it also keeps heading differences unambiguous.
        self.max_step_s = float(max_step_s)
        self.max_speed_mps = float(max_speed_mps)
        self.active = False
        self.reset()

    # -- period control -------------------------------------------------
    def reset(self) -> None:
        """Forget all evidence, motion and commands (a new navigation period)."""
        self.evidence_s = 0.0
        self._anchor: tuple[float, float] | None = None
        self._odom: tuple[float, float, float, float | None] | None = None
        # Commands are (stamp, linear, angular, stream_confirmed_at_receipt).
        self._cmd: tuple | None = None        # latest command this period
        self._step_cmd: tuple | None = None   # command in force at the start of the odometry step
        self._changes: list[tuple] = []       # commands received during the current step
        self._confirmed = False               # a non-zero command has been seen this period
        self._clear_interval()

    def _clear_interval(self) -> None:
        self.interval = dict.fromkeys(_BUCKETS, 0.0)

    def set_active(self, active: bool) -> None:
        """Start (or end) a navigation period; both edges reset the detector."""
        active = bool(active)
        if active != self.active:
            self.reset()
            self.active = active

    # -- inputs -----------------------------------------------------------
    def command(self, stamp: float, linear, angular) -> None:
        """A drive command received at ``stamp`` (ignored while inactive)."""
        if not self.active or not _finite(stamp):
            return
        if _finite(linear, angular) and (float(linear) != 0.0 or float(angular) != 0.0):
            self._confirmed = True  # the Writer itself only ever publishes exact zeros
        value = (float(stamp), linear, angular, self._confirmed)
        self._cmd = value
        if self._odom is None or value[0] <= self._odom[0] + _EPS:
            self._step_cmd = value  # in force from the start of the next step
        else:
            self._changes.append(value)

    def odometry(self, stamp: float, x: float, y: float, yaw: float | None) -> None:
        """An odometry pose (odom frame) received at ``stamp`` (ignored while inactive)."""
        if not self.active or not _finite(stamp, x, y):
            return
        stamp, x, y = float(stamp), float(x), float(y)
        yaw = float(yaw) if yaw is not None and _finite(yaw) else None
        if self._odom is None:
            self._baseline(stamp, x, y, yaw)
            return
        t0, x0, y0, yaw0 = self._odom
        dt = stamp - t0
        if dt < -_EPS:  # clock went backwards (simulation reset): new period
            self.reset()
            self._baseline(stamp, x, y, yaw)
            return
        if dt <= _EPS:
            return
        segments = self._segments(t0, stamp)
        if self._changes:
            self._step_cmd = self._changes[-1]
            self._changes = []
        self._odom = (stamp, x, y, yaw)
        if math.hypot(x - x0, y - y0) > self.max_speed_mps * dt + 0.25:
            # Faster than the robot can drive: an odometry reset or jump, not
            # travel. Re-anchor without clearing evidence.
            self._anchor = (x, y)
            return
        if dt > self.max_step_s:
            return  # a gap: what happened during it cannot be attributed
        dyaw = _angle_diff(yaw, yaw0) if yaw is not None and yaw0 is not None else None
        for duration, kind, angular in segments:
            share = None if dyaw is None else dyaw * duration / dt
            self._attribute(duration, kind, angular, share)
        anchor_x, anchor_y = self._anchor
        if math.hypot(x - anchor_x, y - anchor_y) >= self.min_progress_m:
            self.evidence_s = 0.0
            self._clear_interval()
            self._anchor = (x, y)

    def evaluate(self, now: float) -> bool:
        """Fold the motion seen since the last call into evidence; True once STUCK."""
        if not self.active:
            return False
        bucket = self.interval
        evidence = (bucket["unknown"] + bucket["unconfirmed"] + bucket["invalid"]
                    + bucket["translate"] + bucket["rotate_unverified"])
        verified = bucket["rotate"] - bucket["rotate_unverified"]
        if verified > _EPS:
            needed = max(self.yaw_noise_rad, self.min_rotation_ratio * bucket["rot_expected"])
            if bucket["rot_achieved"] < needed:
                evidence += verified
        self.evidence_s += evidence
        self._clear_interval()
        return self._is_stuck()

    def update(self, now_s: float, x: float, y: float, driving: bool,
               yaw: float | None = None, cmd: tuple[float, float] | None = None) -> bool:
        """Single-sample interface: one odometry pose, then the command in force from now.

        Equivalent to ``set_active(driving)``, ``odometry``, ``command`` and
        ``evaluate`` at the same instant. Without ``cmd`` the detector is
        position-only, as before commands were considered.
        """
        self.set_active(driving)
        if not self.active:
            return False
        self.odometry(now_s, x, y, yaw)
        if cmd is not None:
            try:
                linear, angular = cmd
            except (TypeError, ValueError):
                linear = angular = math.nan
            self.command(now_s, linear, angular)
        return self.evaluate(now_s)

    # -- state --------------------------------------------------------------
    def command_state(self, now: float) -> str:
        return self._classify(self._cmd, now)

    def odom_stale(self, now: float) -> bool:
        return self._odom is None or now - self._odom[0] > self.odom_timeout_s

    def _is_stuck(self) -> bool:
        return self.window_s > 0 and self.evidence_s + _EPS >= self.window_s

    # -- internals ----------------------------------------------------------
    def _baseline(self, stamp, x, y, yaw) -> None:
        self._odom = (stamp, x, y, yaw)
        if self._anchor is None:
            self._anchor = (x, y)

    def _classify(self, cmd, t: float) -> str:
        if cmd is None:
            return UNKNOWN
        stamp, linear, angular, confirmed = cmd
        if not _finite(linear, angular):
            return INVALID
        if not confirmed:
            return UNCONFIRMED  # fresh or stale: zeros alone prove nothing
        if t - stamp > self.cmd_timeout_s + _EPS:
            return STALE
        if abs(float(linear)) >= self.linear_cmd_eps:
            return TRANSLATE
        if abs(float(angular)) >= self.angular_cmd_eps:
            return ROTATE
        return ZERO

    def _segments(self, t0: float, t1: float) -> list[tuple[float, str, float]]:
        """Split [t0, t1] by command changes and staleness: (duration, kind, angular)."""
        result: list[tuple[float, str, float]] = []
        start, cmd = t0, self._step_cmd
        for change in self._changes:
            at = min(max(change[0], t0), t1)
            self._add(result, start, at, cmd)
            start, cmd = at, change
        self._add(result, start, t1, cmd)
        return result

    def _add(self, result, start: float, end: float, cmd) -> None:
        if end - start <= _EPS:
            return
        kind = self._classify(cmd, start)
        angular = float(cmd[2]) if cmd is not None and _finite(cmd[2]) else 0.0
        if kind in (ZERO, TRANSLATE, ROTATE):
            stale_at = cmd[0] + self.cmd_timeout_s
            if start < stale_at < end:
                result.append((stale_at - start, kind, angular))
                result.append((end - stale_at, STALE, 0.0))
                return
        result.append((end - start, kind, angular))

    def _attribute(self, duration: float, kind: str, angular: float, dyaw: float | None) -> None:
        bucket = self.interval
        if kind == UNKNOWN:
            bucket["unknown"] += duration
        elif kind == UNCONFIRMED:
            bucket["unconfirmed"] += duration
        elif kind == INVALID:
            bucket["invalid"] += duration
        elif kind in (STALE, ZERO):
            bucket["idle"] += duration
        elif kind == TRANSLATE:
            bucket["translate"] += duration
        else:
            bucket["rotate"] += duration
            if dyaw is None:
                bucket["rotate_unverified"] += duration
            else:
                bucket["rot_expected"] += abs(angular) * duration
                bucket["rot_achieved"] += math.copysign(1.0, angular) * dyaw
