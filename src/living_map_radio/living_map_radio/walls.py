"""Read RF-blocking rock walls from the Gazebo world, in the Writer map frame.

The radio channel is part of the simulated physics, so it uses the world's
ground-truth geometry, exactly as real rock would attenuate a real signal. No
robot receives this information.
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .model import Wall


def _pose(text: str | None) -> list[float]:
    values = [float(v) for v in (text or "").split()]
    return (values + [0.0] * 6)[:6]


def map_origin(root: ET.Element, robot: str = "writer") -> tuple[float, float]:
    """World position of the Writer spawn, which is the map-frame origin."""
    for element in root.iter("include"):
        if element.findtext("name") == robot:
            x, y, *_ = _pose(element.findtext("pose"))
            return x, y
    raise ValueError(f"robot include '{robot}' not found in world")


def load_walls(world_file: str | Path, prefix: str = "wall") -> list[Wall]:
    root = ET.parse(world_file).getroot()
    origin_x, origin_y = map_origin(root)
    walls: list[Wall] = []
    for model in root.iter("model"):
        if not model.get("name", "").startswith(prefix):
            continue
        x, y, _z, _roll, _pitch, yaw = _pose(model.findtext("pose"))
        size = model.findtext("link/collision/geometry/box/size")
        if size is None:
            continue
        sx, sy, _sz = (float(v) for v in size.split())
        # Rotated walls are represented by their axis-aligned bounding box.
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        walls.append((x - origin_x, y - origin_y, sx * c + sy * s, sx * s + sy * c))
    return walls
