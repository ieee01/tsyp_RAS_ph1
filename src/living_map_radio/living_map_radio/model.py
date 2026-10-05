from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable, Sequence

# A wall is an axis-aligned rectangle in the map frame: (centre_x, centre_y, size_x, size_y).
Wall = tuple[float, float, float, float]

# Log-distance path-loss exponent used to convert wall loss into an equivalent
# extra free-space distance (2.0 = free space; mine drifts behave close to it).
PATH_LOSS_EXPONENT = 2.0
DEFAULT_WALL_LOSS_DB = 8.0


def delivery_probability(distance_m: float, nominal: float = 12.0, max_range: float = 22.0) -> float:
    """Return the configured distance-only packet reception probability.

    The default curve implements the Phase 1 RF abstraction:
    0..12 m => 0.98, 12..18 m => 0.98..0.60,
    18..22 m => 0.60..0.10, and strictly beyond 22 m => 0.
    """
    d = max(0.0, float(distance_m))
    if d <= nominal:
        return 0.98
    if d > max_range:
        return 0.0

    knee = min(18.0, max_range)
    if d <= knee:
        span = max(knee - nominal, 1e-9)
        return 0.98 + (d - nominal) / span * (0.60 - 0.98)

    span = max(max_range - knee, 1e-9)
    return 0.60 + (d - knee) / span * (0.10 - 0.60)


def rssi_like(distance_m: float, walls_crossed: int = 0, wall_loss_db: float = DEFAULT_WALL_LOSS_DB) -> float:
    """Synthetic RSSI-like metric for observability; not a calibrated RF model."""
    return -38.0 - 20.0 * math.log10(max(float(distance_m), 0.25)) - walls_crossed * wall_loss_db


def _segment_hits_box(a: tuple[float, float], b: tuple[float, float], wall: Wall) -> bool:
    """Liang-Barsky clip of segment a-b against an axis-aligned wall box."""
    cx, cy, sx, sy = wall
    xmin, xmax = cx - sx / 2.0, cx + sx / 2.0
    ymin, ymax = cy - sy / 2.0, cy + sy / 2.0
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - xmin), (dx, xmax - a[0]), (-dy, a[1] - ymin), (dy, ymax - a[1])):
        if p == 0.0:
            if q < 0.0:
                return False
            continue
        t = q / p
        if p < 0.0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return True


def walls_crossed(a: tuple[float, float], b: tuple[float, float], walls: Iterable[Wall]) -> int:
    return sum(1 for wall in walls if _segment_hits_box(a, b, wall))


@dataclass(frozen=True)
class LinkQuality:
    distance_m: float
    walls: int
    probability: float
    rssi_dbm: float


class RFModel:
    """Seeded link model: free-space-like decay plus a fixed loss per wall.

    Each wall adds ``wall_loss_db`` of attenuation, which this model expresses
    as an equivalent longer free-space distance on the validated Phase 1
    distance curve. Rock between two radios therefore shortens range sharply,
    which is what makes beacon-to-beacon relaying necessary underground.
    """

    def __init__(self, seed: int = 42, nominal: float = 12.0, max_range: float = 22.0,
                 walls: Sequence[Wall] = (), wall_loss_db: float = DEFAULT_WALL_LOSS_DB):
        self.rng = random.Random(seed)
        self.nominal = nominal
        self.max_range = max_range
        self.walls = list(walls)
        self.wall_loss_db = wall_loss_db

    def link(self, a: tuple[float, float], b: tuple[float, float]) -> LinkQuality:
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        crossed = walls_crossed(a, b, self.walls) if self.walls else 0
        effective = distance * 10.0 ** (crossed * self.wall_loss_db / (10.0 * PATH_LOSS_EXPONENT))
        return LinkQuality(
            distance,
            crossed,
            delivery_probability(effective, self.nominal, self.max_range),
            rssi_like(distance, crossed, self.wall_loss_db),
        )

    def sample(self, probability: float) -> bool:
        return self.rng.random() < probability

    def deliver(self, distance_m: float) -> tuple[bool, float, float]:
        """Distance-only delivery (no walls), kept for the headless validation path."""
        probability = delivery_probability(distance_m, self.nominal, self.max_range)
        return self.rng.random() < probability, probability, rssi_like(distance_m)
