from __future__ import annotations

import heapq
import math
from typing import Callable, Iterable

EdgePredicate = Callable[[int, int], bool]


class MemoryGraph:
    """Small undirected graph reconstructed from persistent beacon memories."""

    def __init__(self) -> None:
        self.nodes: dict[int, dict] = {}
        self.edges: dict[int, dict[int, float]] = {}

    def add(self, node_id: int, x: float, y: float, **attrs) -> None:
        self.nodes[node_id] = {"x": x, "y": y, **attrs}
        self.edges.setdefault(node_id, {})

    def connect(self, a: int, b: int, cost: float | None = None) -> None:
        if a not in self.nodes or b not in self.nodes:
            raise KeyError("unknown node")
        if cost is None:
            cost = math.hypot(
                self.nodes[a]["x"] - self.nodes[b]["x"],
                self.nodes[a]["y"] - self.nodes[b]["y"],
            )
        self.edges[a][b] = cost
        self.edges[b][a] = cost

    def shortest_path(
        self,
        start: int,
        goal: int,
        blocked_types: Iterable[int] = (),
        blocked_node_ids: Iterable[int] = (),
        edge_ok: EdgePredicate | None = None,
    ) -> list[int]:
        blocked = set(blocked_types)
        blocked_nodes = set(blocked_node_ids) - {goal}
        if start in blocked_nodes:
            return []
        queue: list[tuple[float, int, list[int]]] = [(0.0, start, [])]
        best_cost: dict[int, float] = {}

        while queue:
            cost, node_id, path = heapq.heappop(queue)
            if node_id in best_cost and best_cost[node_id] <= cost:
                continue
            best_cost[node_id] = cost
            path = path + [node_id]
            if node_id == goal:
                return path

            for neighbor, edge_cost in self.edges.get(node_id, {}).items():
                if neighbor in blocked_nodes:
                    continue
                if neighbor != goal and self.nodes[neighbor].get("event_type") in blocked:
                    continue
                if edge_ok is not None and not edge_ok(node_id, neighbor):
                    continue
                heapq.heappush(queue, (cost + edge_cost, neighbor, path))

        return []

    def path_cost(self, path: list[int]) -> float:
        """Total edge cost along a node path (0 for a single node)."""
        return sum(self.edges[a][b] for a, b in zip(path, path[1:]))

    def nodes_near_hazards(
        self,
        hazard_ids: Iterable[int],
        radius_m: float,
        target_id: int | None = None,
    ) -> set[int]:
        """Return graph nodes inside any hazard radius, excluding the target."""
        hazards = [self.nodes[node_id] for node_id in hazard_ids if node_id in self.nodes]
        blocked: set[int] = set()
        for node_id, node in self.nodes.items():
            if node_id == target_id:
                continue
            if any(
                math.hypot(node["x"] - hazard["x"], node["y"] - hazard["y"]) <= radius_m
                for hazard in hazards
            ):
                blocked.add(node_id)
        return blocked

    def connect_spatial_neighbors(
        self,
        radius_m: float,
        blocked_node_ids: Iterable[int] = (),
        edge_ok: EdgePredicate | None = None,
        cost: Callable[[int, int, float], float] | None = None,
    ) -> None:
        """Connect every pair of non-blocked nodes within the spatial radius.

        ``edge_ok`` can reject a pair whose straight segment is not traversable,
        for example because it crosses an occupied map cell. Existing edges
        (for example links the Writer actually drove) are kept as they are.
        ``cost`` can weight an inferred edge; by default it costs its length.
        """
        blocked = set(blocked_node_ids)
        candidates = [node_id for node_id in self.nodes if node_id not in blocked]
        for index, first in enumerate(candidates):
            for second in candidates[index + 1 :]:
                if second in self.edges.get(first, {}):
                    continue
                distance = math.hypot(
                    self.nodes[first]["x"] - self.nodes[second]["x"],
                    self.nodes[first]["y"] - self.nodes[second]["y"],
                )
                if distance > radius_m:
                    continue
                if edge_ok is not None and not edge_ok(first, second):
                    continue
                self.connect(first, second, distance if cost is None else cost(first, second, distance))

    def nodes_near_points(self, points: Iterable[tuple[float, float, float]],
                          keep: Iterable[int] = ()) -> set[int]:
        """Return nodes within any (x, y, radius) disc, except those in ``keep``."""
        discs = list(points)
        kept = set(keep)
        return {
            node_id
            for node_id, node in self.nodes.items()
            if node_id not in kept
            and any(math.hypot(node["x"] - x, node["y"] - y) <= r for x, y, r in discs)
        }
