"""Compact the Writer's occupancy grid for the long-distance uplink."""
from __future__ import annotations

from typing import Sequence


def coarse_runs(data: Sequence[int], width: int, height: int, step: int = 4) -> tuple[int, int, list[list[int]]]:
    """Downsample by ``step`` (occupied wins, then free, else unknown) and run-length encode."""
    coarse_width = (width + step - 1) // step
    coarse_height = (height + step - 1) // step
    runs: list[list[int]] = []
    for row in range(coarse_height):
        for column in range(coarse_width):
            values = [data[y * width + x]
                      for y in range(row * step, min((row + 1) * step, height))
                      for x in range(column * step, min((column + 1) * step, width))]
            value = 100 if any(v > 20 for v in values) else (0 if any(v >= 0 for v in values) else -1)
            if runs and runs[-1][0] == value:
                runs[-1][1] += 1
            else:
                runs.append([value, 1])
    return coarse_width, coarse_height, runs


def known_fraction(data: Sequence[int]) -> float:
    return sum(1 for v in data if v >= 0) / len(data) if len(data) else 0.0
