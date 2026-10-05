from __future__ import annotations

import heapq
import random
from dataclasses import dataclass, field


@dataclass(order=True)
class Delivery:
    ready_at: float
    sequence: int
    direction: str = field(compare=False)
    payload: str = field(compare=False)


class LinkModel:
    """Deterministic duplex latency/loss/bandwidth model with outage buffering."""

    def __init__(
        self,
        seed: int = 73,
        latency_ms: float = 600.0,
        jitter_ms: float = 80.0,
        packet_loss: float = 0.02,
        bandwidth_bytes_per_sec: float = 32768.0,
        initially_up: bool = True,
    ) -> None:
        if not 0.0 <= packet_loss <= 1.0:
            raise ValueError("packet_loss must be between 0 and 1")
        if bandwidth_bytes_per_sec <= 0:
            raise ValueError("bandwidth must be positive")
        self.rng = random.Random(seed)
        self.latency_s = max(0.0, latency_ms / 1000.0)
        self.jitter_s = max(0.0, jitter_ms / 1000.0)
        self.packet_loss = packet_loss
        self.bandwidth = bandwidth_bytes_per_sec
        self.up = initially_up
        self.pending: list[Delivery] = []
        self.outage_queue: list[tuple[str, str]] = []
        self.next_available = {"uplink": 0.0, "downlink": 0.0}
        self.sequence = 0
        self.dropped = 0

    def set_up(self, up: bool, now: float) -> None:
        was_down = not self.up
        self.up = bool(up)
        if self.up and was_down:
            queued, self.outage_queue = self.outage_queue, []
            for direction, payload in queued:
                self.submit(direction, payload, now)

    def submit(self, direction: str, payload: str, now: float) -> bool:
        if direction not in self.next_available:
            raise ValueError(f"unknown direction: {direction}")
        if not self.up:
            self.outage_queue.append((direction, payload))
            return True
        if self.rng.random() < self.packet_loss:
            self.dropped += 1
            return False
        transmission_s = len(payload.encode("utf-8")) / self.bandwidth
        start = max(float(now), self.next_available[direction])
        self.next_available[direction] = start + transmission_s
        jitter = self.rng.uniform(-self.jitter_s, self.jitter_s)
        ready = self.next_available[direction] + max(0.0, self.latency_s + jitter)
        self.sequence += 1
        heapq.heappush(self.pending, Delivery(ready, self.sequence, direction, payload))
        return True

    def ready(self, now: float) -> list[Delivery]:
        result: list[Delivery] = []
        while self.pending and self.pending[0].ready_at <= now:
            result.append(heapq.heappop(self.pending))
        return result
