from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ForwardDecision:
    """A snapshot ready for transmission and its outage recovery statistics."""

    payload: str
    discarded_snapshots: int = 0


class LatestSnapshotForwarder:
    """Keep only the latest full-state snapshot while the link is unavailable."""

    def __init__(self, link_up: bool = False) -> None:
        self.link_up = bool(link_up)
        self.pending: str | None = None
        self.superseded = 0

    @property
    def queued_snapshots(self) -> int:
        return 1 if self.pending is not None else 0

    def offer(self, snapshot: str) -> ForwardDecision | None:
        """Forward immediately when online, otherwise replace the pending state."""
        if self.link_up:
            return ForwardDecision(snapshot)
        if self.pending is not None:
            self.superseded += 1
        self.pending = snapshot
        return None

    def set_link(self, link_up: bool) -> ForwardDecision | None:
        """Update link state and release the latest pending snapshot on recovery."""
        restored = bool(link_up) and not self.link_up
        self.link_up = bool(link_up)
        if not restored or self.pending is None:
            return None
        decision = ForwardDecision(self.pending, self.superseded)
        self.pending = None
        self.superseded = 0
        return decision

