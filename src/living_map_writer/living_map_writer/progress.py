"""Detect a Writer that is trying to drive but no longer makes progress."""
from __future__ import annotations

from collections import deque
import math


class StuckDetector:
    """Flags STUCK when, for ``window_s`` of continuous driving intent, the
    Writer stayed within ``min_progress_m`` of where the window started.

    Rotating in place or a short Nav2 recovery does not trigger it; wheels
    spinning in rubble or a robot pinned against debris does.
    """

    def __init__(self, window_s: float = 45.0, min_progress_m: float = 0.3) -> None:
        self.window_s = float(window_s)
        self.min_progress_m = float(min_progress_m)
        self.samples: deque[tuple[float, float, float]] = deque()

    def reset(self) -> None:
        self.samples.clear()

    def update(self, now_s: float, x: float, y: float, driving: bool) -> bool:
        if not driving or self.window_s <= 0:
            self.reset()
            return False
        self.samples.append((now_s, x, y))
        # Keep exactly one sample at least a full window old as the reference.
        while len(self.samples) >= 2 and now_s - self.samples[1][0] >= self.window_s:
            self.samples.popleft()
        start_t, start_x, start_y = self.samples[0]
        if now_s - start_t < self.window_s:
            return False
        moved = max(math.hypot(px - start_x, py - start_y) for _, px, py in self.samples)
        return moved < self.min_progress_m
