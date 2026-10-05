from __future__ import annotations

import math
from collections.abc import Callable, Iterable


Point = tuple[float, float]


def clear_spawn_footprint(data, width: int, height: int, resolution: float,
                          origin: Point, spawn: Point, radius: float) -> list[int]:
    """Remove the known parked robot imprint from its inherited planning map."""
    result = list(data)
    for row in range(max(0, math.floor((spawn[1] - radius - origin[1]) / resolution)),
                     min(height, math.ceil((spawn[1] + radius - origin[1]) / resolution))):
        for column in range(max(0, math.floor((spawn[0] - radius - origin[0]) / resolution)),
                            min(width, math.ceil((spawn[0] + radius - origin[0]) / resolution))):
            point = (origin[0] + (column + 0.5) * resolution,
                     origin[1] + (row + 0.5) * resolution)
            if math.dist(point, spawn) <= radius:
                result[row * width + column] = 0
    return result


def footprint_is_free(point: Point, is_free: Callable[[float, float], bool],
                      radius: float, step: float = 0.1) -> bool:
    """Require known free space throughout a circular turning footprint."""
    cells = math.ceil(radius / step)
    for row in range(-cells, cells + 1):
        for column in range(-cells, cells + 1):
            dx, dy = column * step, row * step
            if dx * dx + dy * dy <= radius * radius and not is_free(point[0] + dx, point[1] + dy):
                return False
    return True


def nearest_clear_waypoint(point: Point, is_clear: Callable[[Point], bool],
                           max_offset: float, preferred_from: Point) -> Point | None:
    """Find a nearby goal where the robot can stop and turn without collision."""
    if is_clear(point):
        return point
    for ring in range(1, math.ceil(max_offset / 0.1) + 1):
        radius = min(ring * 0.1, max_offset)
        candidates = [
            (point[0] + radius * math.cos(index * math.tau / 32),
             point[1] + radius * math.sin(index * math.tau / 32))
            for index in range(32)
        ]
        candidates.sort(key=lambda candidate: math.dist(candidate, preferred_from))
        for candidate in candidates:
            if is_clear(candidate):
                return candidate
    return None


def approach_from_first_in_reach(route: list[Point], labels: list[str], target: Point,
                                 reach: float, stop: float) -> tuple[list[Point], list[str]]:
    """End the route with a straight approach from the first waypoint near the target.

    A fire's standoff beacon sits at about the arrival distance. Stopping exactly on it
    can end just outside that distance, and driving on to a second stop beyond it forces
    a long robot to turn in place at the beacon, where it gets stuck. Instead, the first
    waypoint within ``reach`` is replaced by a stop ``stop`` metres from the target on the
    line from that waypoint, so the robot drives straight in and ends facing the target.
    """
    for index, waypoint in enumerate(route[:-1]):
        distance = math.dist(waypoint, target)
        if distance > reach:
            continue
        if distance <= stop:
            return route[:index + 1], labels[:index + 1]
        t = (distance - stop) / distance
        approach = (waypoint[0] + (target[0] - waypoint[0]) * t, waypoint[1] + (target[1] - waypoint[1]) * t)
        return route[:index] + [approach], labels[:index] + labels[-1:]
    return route, labels


def segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    """Return the shortest distance from a point to a line segment."""
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    length_squared = vx * vx + vy * vy
    if length_squared == 0:
        return math.hypot(px - ax, py - ay)
    projection = max(0.0, min(1.0, (wx * vx + wy * vy) / length_squared))
    closest_x = ax + projection * vx
    closest_y = ay + projection * vy
    return math.hypot(px - closest_x, py - closest_y)


def segment_is_clear(
    start: Point,
    end: Point,
    is_blocked: Callable[[float, float], bool],
    step: float,
    end_margin: float = 0.0,
) -> bool:
    """Sample a straight segment and return false if any sample is blocked.

    Samples within ``end_margin`` of either end are ignored: an edge's endpoint
    is often the mapped object itself (a victim seen by the LiDAR) or a beacon
    dropped beside a wall, which must not make the edge look obstructed.
    """
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    samples = max(1, int(math.ceil(length / max(step, 1e-3))))
    for index in range(samples + 1):
        t = index / samples
        if min(t, 1.0 - t) * length < end_margin:
            continue
        if is_blocked(
            start[0] + t * (end[0] - start[0]),
            start[1] + t * (end[1] - start[1]),
        ):
            return False
    return True


def shortcut_waypoints(
    pose: Point,
    route: list[Point],
    hazards: Iterable[Point],
    clearance: float,
    is_clear: Callable[[Point, Point], bool] | None = None,
) -> list[Point]:
    """Drop leading waypoints the robot can bypass without nearing a hazard.

    Beacon waypoints exist to keep the robot away from known hazards. When
    supplied, ``is_clear`` also checks the bypass against walls. A waypoint is skipped when the straight
    line from the robot to the following waypoint stays ``clearance`` away
    from every hazard, so the robot is not dragged through tight spots (for
    example the Writer's start point) that serve no semantic purpose.
    """
    hazards = list(hazards)
    route = list(route)
    while len(route) >= 2 and (is_clear is None or is_clear(pose, route[1])) and all(
        segment_distance(hx, hy, pose[0], pose[1], route[1][0], route[1][1]) > clearance
        for hx, hy in hazards
    ):
        route.pop(0)
    return route


def _projection(point: Point, start: Point, target: Point) -> float:
    vx = target[0] - start[0]
    vy = target[1] - start[1]
    length_squared = vx * vx + vy * vy
    if length_squared == 0:
        return 0.0
    return (
        (point[0] - start[0]) * vx + (point[1] - start[1]) * vy
    ) / length_squared


def hazard_avoiding_waypoints(
    start: Point,
    target: Point,
    hazards: Iterable[Point],
    clearance: float = 1.5,
    is_free: Callable[[float, float], bool] | None = None,
    on_warning: Callable[[str], None] | None = None,
) -> list[Point]:
    """Create ordered semantic detours, checking both sides against a map.

    When a map callback is supplied, each side is tested at the normal offset
    and at three additional 0.5 m increments. A hazard is skipped only when no
    tested candidate is free.
    """
    start_x, start_y = start
    target_x, target_y = target
    vector_x = target_x - start_x
    vector_y = target_y - start_y
    length = math.hypot(vector_x, vector_y) or 1.0
    perpendicular_x = -vector_y / length
    perpendicular_y = vector_x / length

    relevant = [
        hazard
        for hazard in hazards
        if segment_distance(
            hazard[0], hazard[1], start_x, start_y, target_x, target_y
        ) < clearance
    ]
    relevant.sort(key=lambda hazard: _projection(hazard, start, target))

    detours: list[Point] = []
    for hazard_x, hazard_y in relevant:
        chosen: Point | None = None
        for extra_step in range(4):
            offset = clearance + 0.8 + 0.5 * extra_step
            for side in (1.0, -1.0):
                candidate = (
                    hazard_x + side * perpendicular_x * offset,
                    hazard_y + side * perpendicular_y * offset,
                )
                if is_free is None or is_free(*candidate):
                    chosen = candidate
                    break
            if chosen is not None:
                break
        if chosen is None:
            if on_warning is not None:
                on_warning(
                    f"No free detour found around hazard at ({hazard_x:.2f}, {hazard_y:.2f})"
                )
            continue
        detours.append(chosen)

    detours.sort(key=lambda point: _projection(point, start, target))
    return [*detours, target]
