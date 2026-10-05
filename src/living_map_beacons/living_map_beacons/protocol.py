"""Compact, versioned LivingMap beacon wire protocol.

Frame v2 is 33 bytes in network byte order and answers the three questions the
challenge asks of every beacon:

* WHAT  - ``event_type``, ``severity`` and ``confidence``;
* WHERE - the beacon's own position plus a *guidance vector*
  (``bearing_rad`` / ``range_m``) and the ``previous``/``next`` chain links;
* WHEN  - ``timestamp_sec`` plus the ``ttl_sec`` used by information aging.

A beacon is always dropped where the Writer stands, so its own position is a
place a robot has physically reached. For an event beacon carrying
``FLAG_STANDOFF`` the guidance vector points from that safe spot to the event
("FIRE - 4 m ahead"). For a chain beacon it points to the next beacon deeper
in the mine. Coordinates use centimetres, angles centi-degrees and confidence
0..100 percent.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import struct
import zlib

PROTOCOL_VERSION = 2
# version, flags, mission, beacon, sequence, timestamp, x, y, bearing, range,
# event, severity, confidence, previous, next, ttl | crc32
_FMT_NO_CRC = "!BBHHHIhhhHBBBHHH"
_FMT = _FMT_NO_CRC + "I"
FRAME_SIZE = struct.calcsize(_FMT)

# Role flags (bit field). Several roles may apply to one beacon.
FLAG_JUNCTION = 0x01   # dropped where the passage branches
FLAG_RELAY = 0x02      # dropped to keep the radio chain to the gateway alive
FLAG_STANDOFF = 0x04   # event beacon at a safe spot; guidance vector -> event
FLAG_ENTRANCE = 0x08   # first beacon of the chain: the way out
FLAG_SPACING = 0x10    # spacing fallback breadcrumb
FLAG_RESOLVED = 0x20   # an Executor dealt with the event (victim assisted, fire out)
FLAG_CORRIDOR = 0x40   # place type: a passage with two ways on
FLAG_DEAD_END = 0x80   # place type: a single way on
# Place type: JUNCTION (3+ ways), CORRIDOR (2), DEAD_END (1); none of them = open area.
KNOWN_FLAGS = (FLAG_JUNCTION | FLAG_RELAY | FLAG_STANDOFF | FLAG_ENTRANCE | FLAG_SPACING | FLAG_RESOLVED
               | FLAG_CORRIDOR | FLAG_DEAD_END)

ROLE_NAMES = {
    FLAG_ENTRANCE: "ENTRANCE",
    FLAG_JUNCTION: "JUNCTION",
    FLAG_RELAY: "RELAY",
    FLAG_STANDOFF: "STANDOFF",
    FLAG_SPACING: "BREADCRUMB",
    FLAG_RESOLVED: "RESOLVED",
    FLAG_CORRIDOR: "CORRIDOR",
    FLAG_DEAD_END: "DEAD END",
}

MAX_RANGE_M = 655.35


@dataclass(frozen=True)
class BeaconPacket:
    protocol_version: int = PROTOCOL_VERSION
    flags: int = 0
    mission_id: int = 1
    beacon_id: int = 0
    sequence: int = 0
    timestamp_sec: int = 0
    x_m: float = 0.0
    y_m: float = 0.0
    bearing_rad: float = 0.0
    range_m: float = 0.0
    event_type: int = 0
    severity: int = 0
    confidence: float = 1.0
    previous_beacon_id: int = 0
    next_beacon_id: int = 0
    ttl_sec: int = 1800
    crc32: int = 0


def _clamp_int(v: int, lo: int, hi: int, name: str) -> int:
    if not lo <= v <= hi:
        raise ValueError(f"{name}={v} outside [{lo},{hi}]")
    return v


def encode(p: BeaconPacket) -> bytes:
    if p.protocol_version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version {p.protocol_version}")
    x = _clamp_int(round(p.x_m * 100), -32768, 32767, "x_cm")
    y = _clamp_int(round(p.y_m * 100), -32768, 32767, "y_cm")
    bearing_deg = ((math.degrees(p.bearing_rad) + 180.0) % 360.0) - 180.0
    bearing = _clamp_int(round(bearing_deg * 100), -18000, 18000, "bearing_cdeg")
    if not math.isfinite(p.range_m):
        raise ValueError("range_m must be finite")
    range_cm = _clamp_int(round(p.range_m * 100), 0, 65535, "range_cm")
    conf = _clamp_int(round(p.confidence * 100), 0, 100, "confidence_percent")
    fields = (
        p.protocol_version,
        _clamp_int(p.flags, 0, 255, "flags"),
        _clamp_int(p.mission_id, 0, 65535, "mission_id"),
        _clamp_int(p.beacon_id, 0, 65535, "beacon_id"),
        _clamp_int(p.sequence, 0, 65535, "sequence"),
        _clamp_int(p.timestamp_sec, 0, 2**32 - 1, "timestamp_sec"),
        x,
        y,
        bearing,
        range_cm,
        _clamp_int(p.event_type, 0, 255, "event_type"),
        _clamp_int(p.severity, 0, 255, "severity"),
        conf,
        _clamp_int(p.previous_beacon_id, 0, 65535, "previous_beacon_id"),
        _clamp_int(p.next_beacon_id, 0, 65535, "next_beacon_id"),
        _clamp_int(p.ttl_sec, 0, 65535, "ttl_sec"),
    )
    body = struct.pack(_FMT_NO_CRC, *fields)
    crc = zlib.crc32(body) & 0xFFFFFFFF
    return body + struct.pack("!I", crc)


def validate_crc(data: bytes) -> bool:
    return len(data) == FRAME_SIZE and (zlib.crc32(data[:-4]) & 0xFFFFFFFF) == struct.unpack("!I", data[-4:])[0]


def decode(data: bytes) -> BeaconPacket:
    if len(data) != FRAME_SIZE:
        raise ValueError(f"expected {FRAME_SIZE} bytes, got {len(data)}")
    if not validate_crc(data):
        raise ValueError("CRC32 mismatch")
    v, flags, mission, bid, seq, ts, x, y, bearing, rng, etype, sev, conf, prev, nxt, ttl, crc = struct.unpack(_FMT, data)
    if v != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version {v}")
    return BeaconPacket(v, flags, mission, bid, seq, ts, x / 100.0, y / 100.0, math.radians(bearing / 100.0),
                        rng / 100.0, etype, sev, conf / 100.0, prev, nxt, ttl, crc)


def roles(flags: int) -> list[str]:
    """Human-readable role names, entrance first."""
    return [name for bit, name in ROLE_NAMES.items() if flags & bit]


def place_flags(openings: int) -> int:
    """Encode the place type seen around a beacon from its number of openings."""
    if openings >= 3:
        return FLAG_JUNCTION
    return {2: FLAG_CORRIDOR, 1: FLAG_DEAD_END}.get(openings, 0)


def resolve_update(current: BeaconPacket, timestamp_sec: int) -> BeaconPacket:
    """The write request a robot sends: its copy of the memory, marked resolved."""
    if current.event_type == 0:
        raise ValueError("navigation beacons have no event to resolve")
    return replace(current, flags=current.flags | FLAG_RESOLVED, sequence=(current.sequence + 1) % 65536,
                   timestamp_sec=timestamp_sec, crc32=0)


def apply_resolve(stored: BeaconPacket, request: BeaconPacket, sequence: int) -> BeaconPacket | None:
    """What a beacon does with a write request: the only change a robot may make.

    The request must name the same event memory (mission, beacon, event type and
    position). The beacon then marks *its own* current copy resolved, so a robot
    can never move, re-type or truncate a memory, and links the beacon learned
    after the robot read it are kept. Returns None for an invalid request.
    """
    # Compare at wire precision: the stored copy may hold the exact drop position,
    # while the request was decoded from a frame rounded to centimetres.
    same_memory = (
        all(getattr(request, name) == getattr(stored, name) for name in ("mission_id", "beacon_id", "event_type"))
        and all(round(getattr(request, name) * 100) == round(getattr(stored, name) * 100) for name in ("x_m", "y_m"))
    )
    if stored.event_type == 0 or not same_memory or not request.flags & FLAG_RESOLVED:
        return None
    return replace(stored, flags=stored.flags | FLAG_RESOLVED, sequence=sequence,
                   timestamp_sec=max(stored.timestamp_sec, request.timestamp_sec), crc32=0)


def target_position(packet: BeaconPacket) -> tuple[float, float]:
    """Where the beacon's WHAT is located.

    A standoff event beacon describes something ``range_m`` away along
    ``bearing_rad``; every other beacon describes its own position.
    """
    if packet.flags & FLAG_STANDOFF and packet.event_type != 0:
        return (packet.x_m + packet.range_m * math.cos(packet.bearing_rad),
                packet.y_m + packet.range_m * math.sin(packet.bearing_rad))
    return packet.x_m, packet.y_m


def guidance(from_xy: tuple[float, float], to_xy: tuple[float, float]) -> tuple[float, float]:
    """Return (bearing_rad, range_m) from one map point to another, wire-safe."""
    dx, dy = to_xy[0] - from_xy[0], to_xy[1] - from_xy[1]
    return math.atan2(dy, dx), min(math.hypot(dx, dy), MAX_RANGE_M)
