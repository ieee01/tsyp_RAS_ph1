"""Battery model and the energy-aware decision to turn back.

The robot drains energy while powered (computer, LiDAR, radio) and per metre
driven. It must keep enough charge to drive home: the reserve is the energy for
the estimated route home (straight line × tortuosity), times a safety factor,
plus a fixed floor.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BatterySpec:
    idle_pct_per_s: float = 0.03     # electronics, LiDAR, radio
    drive_pct_per_m: float = 0.3     # motors
    tortuosity: float = 1.6          # tunnel route home vs straight line
    safety_factor: float = 1.5
    floor_pct: float = 10.0


class Battery:
    def __init__(self, spec: BatterySpec | None = None, level_pct: float = 100.0) -> None:
        self.spec = spec or BatterySpec()
        self.level = max(0.0, min(100.0, float(level_pct)))
        self._last: tuple[float, float] | None = None

    @property
    def depleted(self) -> bool:
        return self.level <= 0.0

    def update(self, dt_s: float, position: tuple[float, float] | None) -> float:
        """Drain for ``dt_s`` seconds powered plus the distance moved since the last update."""
        moved = 0.0
        if position is not None:
            if self._last is not None:
                moved = math.dist(self._last, position)
                if moved > 2.0:  # a pose jump (localisation correction) is not travel
                    moved = 0.0
            self._last = position
        drain = max(0.0, dt_s) * self.spec.idle_pct_per_s + moved * self.spec.drive_pct_per_m
        self.level = max(0.0, self.level - drain)
        return self.level

    def energy_home(self, position: tuple[float, float], home: tuple[float, float], speed_mps: float = 0.4) -> float:
        """Estimated charge (percent) needed to drive home from ``position``."""
        distance = math.dist(position, home) * self.spec.tortuosity
        return distance * self.spec.drive_pct_per_m + distance / max(speed_mps, 0.05) * self.spec.idle_pct_per_s

    def reserve(self, position: tuple[float, float], home: tuple[float, float]) -> float:
        return self.spec.floor_pct + self.spec.safety_factor * self.energy_home(position, home)

    def must_return(self, position: tuple[float, float], home: tuple[float, float]) -> bool:
        return self.level <= self.reserve(position, home)
