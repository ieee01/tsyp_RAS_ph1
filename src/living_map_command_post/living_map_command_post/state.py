from __future__ import annotations

import copy
import threading
import time


class CommandPostState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.received_at = None
        self._snapshot = {
            "memories": [], "robots": {}, "mission": {"status": "LINK WAIT"},
            "network": {"long_distance_link": "WAITING"}, "elapsed_s": 0.0,
        }

    def replace(self, snapshot: dict) -> None:
        with self._lock:
            update = copy.deepcopy(snapshot)
            # The gateway omits an unchanged map to save uplink bandwidth.
            if "map" not in update and update.get("map_version") is not None \
                    and update.get("map_version") == self._snapshot.get("map_version"):
                update["map"] = self._snapshot.get("map")
            self._snapshot = update
            self.received_at = time.monotonic()

    def snapshot(self) -> dict:
        with self._lock:
            snapshot = copy.deepcopy(self._snapshot)
            age = time.monotonic() - self.received_at if self.received_at is not None else None
            if age is not None and "generated_at" in snapshot:
                age = max(age, time.time() - snapshot["generated_at"])
            snapshot["received_age_s"] = age
            return snapshot

    def mark_mission_sent(self, mission_id: int) -> None:
        with self._lock:
            self._snapshot.setdefault("mission", {})
            self._snapshot["mission"] = {"status": "UPLINK SENT", "mission_id": mission_id}
