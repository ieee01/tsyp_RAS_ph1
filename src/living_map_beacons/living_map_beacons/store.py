from __future__ import annotations

import json
import os
from dataclasses import asdict
from pathlib import Path
from typing import Mapping

from .protocol import PROTOCOL_VERSION, BeaconPacket


class BeaconStore:
    """Crash-tolerant JSON persistence for deployed beacon memories.

    Writes are performed to a temporary file, flushed and fsynced, then atomically
    renamed over the live file. The previous valid file is preserved as `.bak` so
    startup can recover from an interrupted or externally corrupted primary file.
    """

    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.backup_path = self.path.with_suffix(self.path.suffix + ".bak")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.recovered_from_backup = False

    @staticmethod
    def _decode(text: str) -> dict[int, BeaconPacket]:
        data = json.loads(text)
        if not isinstance(data, dict):
            raise ValueError("beacon store root must be a JSON object")
        return {int(key): BeaconPacket(**BeaconStore._upgrade(value)) for key, value in data.items()}

    @staticmethod
    def _upgrade(value: dict) -> dict:
        """Read memories persisted by protocol v1 (yaw field, no range/roles)."""
        if not isinstance(value, dict):
            raise ValueError("beacon record must be a JSON object")
        if int(value.get("protocol_version", PROTOCOL_VERSION)) == 1:
            value = dict(value)
            value["bearing_rad"] = value.pop("yaw_rad", 0.0)
            value["protocol_version"] = PROTOCOL_VERSION
        return value

    def load(self) -> dict[int, BeaconPacket]:
        self.recovered_from_backup = False
        if not self.path.exists():
            if not self.backup_path.exists():
                return {}
            try:
                result = self._decode(self.backup_path.read_text())
                self.recovered_from_backup = True
                return result
            except (OSError, UnicodeDecodeError, ValueError, TypeError, json.JSONDecodeError):
                self._quarantine(self.backup_path)
                return {}

        try:
            return self._decode(self.path.read_text())
        except (OSError, UnicodeDecodeError, ValueError, TypeError, json.JSONDecodeError):
            if self.backup_path.exists():
                try:
                    result = self._decode(self.backup_path.read_text())
                    self.recovered_from_backup = True
                    self._quarantine(self.path)
                    return result
                except (OSError, UnicodeDecodeError, ValueError, TypeError, json.JSONDecodeError):
                    self._quarantine(self.backup_path)
            self._quarantine(self.path)
            return {}

    @staticmethod
    def _quarantine(path: Path) -> None:
        if not path.exists():
            return
        candidate = path.with_name(path.name + ".corrupt")
        counter = 1
        while candidate.exists():
            candidate = path.with_name(path.name + f".corrupt.{counter}")
            counter += 1
        try:
            os.replace(path, candidate)
        except OSError:
            pass

    def _atomic_write_bytes(self, destination: Path, payload: bytes) -> None:
        temporary = destination.with_name(destination.name + ".tmp")
        with temporary.open("wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, destination)
        # Ensure the directory entry itself is durable on POSIX filesystems.
        try:
            directory_fd = os.open(destination.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except OSError:
            # Some filesystems (notably VM shared folders) do not allow fsync on
            # directories. The atomic rename still prevents partial JSON files.
            pass

    def save(self, items: Mapping[int, BeaconPacket]) -> None:
        payload = json.dumps(
            {str(key): asdict(value) for key, value in items.items()},
            indent=2,
            sort_keys=True,
        ).encode("utf-8")

        # Preserve the last complete primary state before replacing it.
        if self.path.exists():
            try:
                current = self.path.read_bytes()
                self._decode(current.decode("utf-8"))
                self._atomic_write_bytes(self.backup_path, current)
            except (OSError, UnicodeDecodeError, ValueError, TypeError, json.JSONDecodeError):
                # Never overwrite a known-good backup with a corrupt primary.
                pass

        self._atomic_write_bytes(self.path, payload)
