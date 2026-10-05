"""Writer sortie lifecycle: explore, then come home while it still can.

EXPLORING -> RETURNING when no reachable frontier remains for a few
consecutive checks (the mine is mapped) or the exploration budget - a stand-in
for the battery reserve needed to drive back - is spent.
RETURNING -> HOME on arrival, or RETURN_FAILED after repeated failed attempts.
"""
from __future__ import annotations


class Sortie:
    def __init__(self, budget_s: float = 420.0, empty_ticks_to_complete: int = 4,
                 max_return_attempts: int = 3) -> None:
        self.budget_s = float(budget_s)
        self.empty_ticks_to_complete = max(1, int(empty_ticks_to_complete))
        self.max_return_attempts = max(1, int(max_return_attempts))
        self.phase = "EXPLORING"
        self.reason = ""
        self.explored_s = 0.0
        self.empty_ticks = 0
        self.return_attempts_failed = 0

    def spend(self, seconds: float) -> None:
        self.explored_s += max(0.0, float(seconds))

    def budget_spent(self) -> bool:
        return self.budget_s > 0 and self.explored_s >= self.budget_s

    def frontier_found(self) -> None:
        self.empty_ticks = 0

    def no_frontier(self) -> bool:
        """Record a check with no reachable frontier; True when this completes exploration."""
        if self.phase != "EXPLORING":
            return False
        self.empty_ticks += 1
        if self.empty_ticks >= self.empty_ticks_to_complete:
            self.start_return("no reachable frontier left - mine mapped")
            return True
        return False

    def start_return(self, reason: str) -> None:
        if self.phase == "EXPLORING":
            self.phase = "RETURNING"
            self.reason = reason

    def return_failed(self) -> bool:
        """Count a failed return attempt; True once the Writer must give up."""
        if self.phase != "RETURNING":
            return False
        self.return_attempts_failed += 1
        if self.return_attempts_failed >= self.max_return_attempts:
            self.phase = "RETURN_FAILED"
            return True
        return False

    def arrived_home(self) -> None:
        if self.phase == "RETURNING":
            self.phase = "HOME"
