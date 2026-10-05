"""Beacon-to-beacon radio mesh between the disconnected mine and the gateway.

Every deployed beacon is a small store-and-forward radio. A beacon's periodic
advertisement is flooded: each live beacon that hears a frame for the first
time in that advertising round re-transmits it once, so a memory written deep
in the mine reaches the gateway hop by hop even when no direct path exists.
Frames are relayed unmodified, so the originator's CRC32 protects the payload
end to end.

Robot telemetry (Writer/Executor status, the Writer's map and the Executor's
beacon confirmations) is unicast along the best route instead: the route that
maximises end-to-end delivery probability, with link-layer retries per hop.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math

from .model import LinkQuality, RFModel

GATEWAY = "gateway"
Point = tuple[float, float]


@dataclass
class FloodResult:
    """Outcome of one advertising round of one beacon frame."""

    gateway_hops: int | None = None
    gateway_path: list[int] = field(default_factory=list)
    transmissions: int = 0
    attempts: list[tuple[str, str, LinkQuality, bool]] = field(default_factory=list)

    @property
    def delivered(self) -> bool:
        return self.gateway_hops is not None


@dataclass(frozen=True)
class Route:
    """Best route to the gateway: beacon IDs that transmit, in order, and radio hops."""

    path: tuple[int, ...]
    probability: float
    hops: int


class RadioMesh:
    def __init__(self, model: RFModel, gateway: Point, min_link_probability: float = 0.3,
                 link_retries: int = 3, max_hops: int = 16) -> None:
        self.model = model
        self.gateway = gateway
        self.min_link_probability = min_link_probability
        self.link_retries = max(1, int(link_retries))
        self.max_hops = max_hops
        self.beacons: dict[int, Point] = {}
        self.dead: set[int] = set()
        self._links: dict[tuple, LinkQuality] = {}

    # -- membership -------------------------------------------------------
    def update_beacon(self, beacon_id: int, position: Point) -> bool:
        """Register a beacon heard on the air. Returns True when it is new or moved."""
        if self.beacons.get(beacon_id) == position:
            return False
        self.beacons[beacon_id] = position
        self._links = {key: value for key, value in self._links.items() if beacon_id not in key}
        return True

    def kill(self, beacon_id: int) -> None:
        self.dead.add(beacon_id)

    def clear(self) -> None:
        self.beacons.clear()
        self.dead.clear()
        self._links.clear()

    def alive(self) -> list[int]:
        return sorted(bid for bid in self.beacons if bid not in self.dead)

    def position(self, node) -> Point:
        return self.gateway if node == GATEWAY else self.beacons[node]

    # -- link quality -----------------------------------------------------
    def link(self, a, b) -> LinkQuality:
        """Cached quality between two fixed mesh nodes (beacon IDs or the gateway)."""
        key = (a, b) if str(a) <= str(b) else (b, a)
        quality = self._links.get(key)
        if quality is None:
            quality = self.model.link(self.position(a), self.position(b))
            self._links[key] = quality
        return quality

    def effective_probability(self, probability: float) -> float:
        """Per-hop delivery with link-layer retries (ARQ)."""
        return 1.0 - (1.0 - probability) ** self.link_retries

    def usable(self, probability: float) -> bool:
        return probability >= self.min_link_probability

    # -- routing ----------------------------------------------------------
    def routes(self) -> dict[int, Route]:
        """Most reliable route from every live beacon to the gateway (Dijkstra on -log p)."""
        alive = self.alive()
        best: dict[int, tuple[float, tuple[int, ...]]] = {}
        queue: list[tuple[float, int, tuple[int, ...]]] = []
        for beacon_id in alive:
            quality = self.link(beacon_id, GATEWAY)
            if self.usable(quality.probability):
                cost = -math.log(max(self.effective_probability(quality.probability), 1e-9))
                heapq.heappush(queue, (cost, beacon_id, (beacon_id,)))
        while queue:
            cost, node, path = heapq.heappop(queue)
            if node in best:
                continue
            best[node] = (cost, path)
            if len(path) >= self.max_hops:
                continue
            for other in alive:
                if other in best or other == node:
                    continue
                quality = self.link(node, other)
                if not self.usable(quality.probability):
                    continue
                step = -math.log(max(self.effective_probability(quality.probability), 1e-9))
                heapq.heappush(queue, (cost + step, other, (other,) + path))
        return {node: Route(path, math.exp(-cost), len(path)) for node, (cost, path) in best.items()}

    def route_from(self, point: Point, routes: dict[int, Route] | None = None) -> Route | None:
        """Best route for a robot at ``point``: direct to the gateway or via one beacon."""
        routes = self.routes() if routes is None else routes
        best: Route | None = None
        direct = self.model.link(point, self.gateway)
        if self.usable(direct.probability):
            best = Route((), self.effective_probability(direct.probability), 1)
        for beacon_id, route in routes.items():
            quality = self.model.link(point, self.beacons[beacon_id])
            if not self.usable(quality.probability):
                continue
            probability = self.effective_probability(quality.probability) * route.probability
            if best is None or probability > best.probability:
                best = Route(route.path, probability, route.hops + 1)
        return best

    def uplink_quality(self, point: Point, routes: dict[int, Route] | None = None) -> float:
        """Raw first-hop probability from ``point`` into the gateway-connected mesh.

        The Writer measures this as the signal of the best beacon it can still
        hear that has a path out; it drops a relay beacon before it falls away.
        """
        routes = self.routes() if routes is None else routes
        best = self.model.link(point, self.gateway).probability
        for beacon_id in routes:
            best = max(best, self.model.link(point, self.beacons[beacon_id]).probability)
        return best

    def send_unicast(self, point: Point, routes: dict[int, Route] | None = None) -> tuple[bool, int | None]:
        """Sample one telemetry delivery from ``point`` along its best route."""
        route = self.route_from(point, routes)
        if route is None:
            return False, None
        hops: list[Point] = [point] + [self.beacons[b] for b in route.path] + [self.gateway]
        for a, b in zip(hops, hops[1:]):
            probability = self.model.link(a, b).probability
            if not any(self.model.sample(probability) for _ in range(self.link_retries)):
                return False, route.hops
        return True, route.hops

    # -- flooding ---------------------------------------------------------
    def flood(self, origin: int) -> FloodResult:
        """Flood one advertisement of ``origin``'s frame through live beacons."""
        result = FloodResult()
        if origin in self.dead or origin not in self.beacons:
            return result
        relayed = {origin}
        parent: dict[int, int | None] = {origin: None}
        frontier: list[tuple[int, int]] = [(origin, 1)]
        while frontier:
            next_frontier: list[tuple[int, int]] = []
            for transmitter, hops in frontier:
                result.transmissions += 1
                if result.gateway_hops is None:
                    quality = self.link(transmitter, GATEWAY)
                    delivered = self.model.sample(quality.probability)
                    result.attempts.append((str(transmitter), GATEWAY, quality, delivered))
                    if delivered:
                        result.gateway_hops = hops
                        path, node = [], transmitter
                        while node is not None:
                            path.append(node)
                            node = parent[node]
                        result.gateway_path = list(reversed(path))
                if hops >= self.max_hops:
                    continue
                for receiver in self.alive():
                    if receiver in relayed:
                        continue
                    quality = self.link(transmitter, receiver)
                    if quality.probability <= 0.0:
                        continue
                    if self.model.sample(quality.probability):
                        relayed.add(receiver)
                        parent[receiver] = transmitter
                        next_frontier.append((receiver, hops + 1))
            frontier = next_frontier
        return result

    # -- observability ----------------------------------------------------
    def topology(self, robots: dict[str, Point] | None = None) -> dict:
        routes = self.routes()
        alive = self.alive()
        links = []
        nodes = alive + [GATEWAY]
        for index, a in enumerate(nodes):
            for b in nodes[index + 1:]:
                quality = self.link(a, b)
                if self.usable(quality.probability):
                    links.append({"a": a, "b": b, "p": round(quality.probability, 3),
                                  "walls": quality.walls, "rssi": round(quality.rssi_dbm, 1)})
        robot_routes = {}
        for name, point in (robots or {}).items():
            route = self.route_from(point, routes)
            robot_routes[name] = {
                "connected": route is not None,
                "path": [] if route is None else list(route.path),
                "hops": None if route is None else route.hops,
                "p": 0.0 if route is None else round(route.probability, 3),
                "uplink_p": round(self.uplink_quality(point, routes), 3),
            }
        return {
            "gateway": list(self.gateway),
            "beacons": {str(b): list(self.beacons[b]) for b in sorted(self.beacons)},
            "dead": sorted(self.dead),
            "links": links,
            "routes": {str(b): {"path": list(r.path), "hops": r.hops, "p": round(r.probability, 3)}
                       for b, r in sorted(routes.items())},
            "unreachable": [b for b in alive if b not in routes],
            "robots": robot_routes,
        }
