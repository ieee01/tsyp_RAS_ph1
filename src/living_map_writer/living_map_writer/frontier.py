from __future__ import annotations

from collections import deque
import math


def frontier_cells(data, width, height, free_max=20):
    return [
        (x, y)
        for y in range(1, height - 1)
        for x in range(1, width - 1)
        if 0 <= data[y * width + x] <= free_max
        and any(data[(y + dy) * width + x + dx] < 0
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)))
    ]


def clusters(cells):
    remaining = set(cells)
    result = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        queue = [seed]
        cluster = [seed]
        while queue:
            x, y = queue.pop()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                neighbor = (x + dx, y + dy)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
                    cluster.append(neighbor)
        result.append(cluster)
    return result


def choose_frontier(data, width, height, resolution, origin_x, origin_y,
                    robot_x, robot_y, min_cluster=5, blacklist=(), clearance=0.35,
                    visited=(), visited_radius=1.0, min_x=None):
    """Choose an actual frontier cell reachable through free, robot-sized space.

    ``visited`` holds frontier goals the robot already reached. Only cells
    within ``visited_radius`` of these goals are excluded. Cluster the remaining
    cells so a visited section cannot hide a connected, unexplored boundary.
    ``min_x`` bounds exploration to the mine side of the entrance: the area
    behind it is the Outside Network Area, not the disconnected zone.
    """
    radius = math.ceil(clearance / resolution)
    offsets = [(dx, dy) for dy in range(-radius, radius + 1)
               for dx in range(-radius, radius + 1)
               if math.hypot(dx, dy) * resolution <= clearance]
    blocked = set()
    for index, value in enumerate(data):
        if value > 20:
            x, y = index % width, index // width
            blocked.update((x + dx, y + dy) for dx, dy in offsets)
    free = {(i % width, i // width) for i, value in enumerate(data)
            if 0 <= value <= 20 and (i % width, i // width) not in blocked}
    start = (math.floor((robot_x - origin_x) / resolution),
             math.floor((robot_y - origin_y) / resolution))
    if start not in free:
        # SLAM can leave a small unknown patch beneath the laser/base.
        if not free:
            return None
        nearest = min(free, key=lambda p: ((p[0] - start[0])**2 + (p[1] - start[1])**2, p))
        if math.dist(start, nearest) * resolution > clearance:
            return None
        start = nearest
    reachable = {start}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (x + dx, y + dy)
            if neighbor in free and neighbor not in reachable:
                reachable.add(neighbor)
                queue.append(neighbor)

    unvisited_cells = [
        (x, y) for x, y in frontier_cells(data, width, height)
        if (min_x is None or origin_x + (x + 0.5) * resolution >= min_x)
        and not any(
            (origin_x + (x + 0.5) * resolution - vx)**2
            + (origin_y + (y + 0.5) * resolution - vy)**2 <= visited_radius**2
            for vx, vy in visited
        )
    ]
    best = None
    for cluster in clusters(unvisited_cells):
        if len(cluster) < min_cluster:
            continue
        cx = sum(x for x, _ in cluster) / len(cluster)
        cy = sum(y for _, y in cluster) / len(cluster)
        eligible = []
        for x, y in cluster:
            wx = origin_x + (x + 0.5) * resolution
            wy = origin_y + (y + 0.5) * resolution
            if (x, y) in reachable and not any(
                (wx - bx)**2 + (wy - by)**2 < 0.6**2 for bx, by in blacklist
            ) and math.hypot(wx - robot_x, wy - robot_y) >= 0.5:
                eligible.append((x, y))
        if not eligible:
            continue
        x, y = min(eligible, key=lambda p: ((p[0] - cx)**2 + (p[1] - cy)**2, p))
        wx = origin_x + (x + 0.5) * resolution
        wy = origin_y + (y + 0.5) * resolution
        score = len(eligible) * resolution - 0.7 * math.hypot(wx - robot_x, wy - robot_y)
        if best is None or score > best[0]:
            best = (score, wx, wy, len(cluster))
    return best
