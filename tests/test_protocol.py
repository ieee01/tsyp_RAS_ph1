import json
import math

import pytest

from living_map_beacons.protocol import (FLAG_JUNCTION, FLAG_STANDOFF, FRAME_SIZE, BeaconPacket, decode, encode,
                                         guidance, roles, target_position, validate_crc)
from living_map_beacons.store import BeaconStore


def test_round_trip_and_size():
    p = BeaconPacket(beacon_id=7, sequence=3, timestamp_sec=1234, x_m=12.34, y_m=-3.21, bearing_rad=1.2,
                     range_m=4.25, event_type=1, severity=4, confidence=.94, ttl_sec=900,
                     flags=FLAG_STANDOFF | FLAG_JUNCTION, next_beacon_id=8, previous_beacon_id=6)
    raw = encode(p)
    q = decode(raw)
    assert len(raw) == FRAME_SIZE == 33
    assert q.beacon_id == 7
    assert q.x_m == pytest.approx(12.34, abs=.01)
    assert q.bearing_rad == pytest.approx(1.2, abs=1e-3)
    assert q.range_m == pytest.approx(4.25, abs=.01)
    assert q.confidence == pytest.approx(.94, abs=.01)
    assert (q.previous_beacon_id, q.next_beacon_id) == (6, 8)
    assert roles(q.flags) == ["JUNCTION", "STANDOFF"]


def test_crc_detects_corruption():
    raw = bytearray(encode(BeaconPacket(beacon_id=1)))
    raw[5] ^= 0x01
    assert not validate_crc(bytes(raw))
    with pytest.raises(ValueError):
        decode(bytes(raw))


def test_range_and_version():
    with pytest.raises(ValueError):
        encode(BeaconPacket(x_m=1000))
    with pytest.raises(ValueError):
        encode(BeaconPacket(protocol_version=1))
    with pytest.raises(ValueError, match="range"):
        encode(BeaconPacket(range_m=700))
    with pytest.raises(ValueError):
        encode(BeaconPacket(range_m=-1))


def test_out_of_range_flags_raise_value_error():
    with pytest.raises(ValueError, match="flags"):
        encode(BeaconPacket(flags=256))


def test_standoff_beacon_points_at_its_event():
    bearing, distance = guidance((2.0, 1.0), (5.0, 5.0))
    packet = decode(encode(BeaconPacket(x_m=2.0, y_m=1.0, bearing_rad=bearing, range_m=distance,
                                        event_type=4, flags=FLAG_STANDOFF)))
    assert distance == pytest.approx(5.0)
    assert target_position(packet) == pytest.approx((5.0, 5.0), abs=0.02)


def test_chain_and_nav_beacons_describe_their_own_position():
    nav = BeaconPacket(x_m=1.0, y_m=2.0, bearing_rad=math.pi / 2, range_m=6.0)
    unflagged_event = BeaconPacket(x_m=1.0, y_m=2.0, bearing_rad=0.3, range_m=3.0, event_type=1)
    assert target_position(nav) == (1.0, 2.0)
    assert target_position(unflagged_event) == (1.0, 2.0)


def test_store_upgrades_protocol_v1_memories(tmp_path):
    path = tmp_path / "beacons.json"
    path.write_text(json.dumps({"3": {
        "protocol_version": 1, "flags": 0, "mission_id": 1, "beacon_id": 3, "sequence": 4, "timestamp_sec": 9,
        "x_m": 1.5, "y_m": -2.0, "yaw_rad": 0.5, "event_type": 2, "severity": 4, "confidence": 0.9,
        "previous_beacon_id": 2, "next_beacon_id": 0, "ttl_sec": 1800, "crc32": 0}}))
    packet = BeaconStore(str(path)).load()[3]
    assert packet.protocol_version == 2
    assert packet.bearing_rad == 0.5
    assert len(encode(packet)) == FRAME_SIZE


def test_robot_can_only_mark_an_event_resolved():
    from living_map_beacons.protocol import FLAG_RESOLVED, apply_resolve, resolve_update
    from dataclasses import replace
    stored = BeaconPacket(beacon_id=4, sequence=7, timestamp_sec=100, x_m=3.0, y_m=-1.0, event_type=4,
                          flags=FLAG_STANDOFF, bearing_rad=0.4, range_m=2.5, next_beacon_id=5)
    request = decode(encode(resolve_update(stored, 200)))
    assert request.flags & FLAG_RESOLVED and request.sequence == 8
    # The beacon learned a newer link after the robot read it: the link survives.
    newer = replace(stored, next_beacon_id=9, sequence=11)
    applied = apply_resolve(newer, request, 12)
    assert applied.flags == FLAG_STANDOFF | FLAG_RESOLVED
    assert (applied.next_beacon_id, applied.sequence, applied.timestamp_sec) == (9, 12, 200)
    assert (applied.x_m, applied.range_m) == (stored.x_m, stored.range_m)
    # A request for a different memory, a moved one, or a non-resolve is refused.
    assert apply_resolve(newer, replace(request, beacon_id=5), 12) is None
    assert apply_resolve(newer, replace(request, x_m=9.0), 12) is None
    assert apply_resolve(newer, replace(request, flags=FLAG_STANDOFF), 12) is None
    with pytest.raises(ValueError):
        resolve_update(BeaconPacket(event_type=0), 1)


def test_resolve_matches_a_stored_copy_with_exact_coordinates():
    """Regression: the beacon store keeps unrounded positions; requests arrive rounded."""
    from living_map_beacons.protocol import FLAG_RESOLVED, apply_resolve, resolve_update
    stored = BeaconPacket(beacon_id=2, sequence=5, timestamp_sec=10, x_m=3.891234, y_m=-2.214567, event_type=1,
                          flags=FLAG_STANDOFF, bearing_rad=0.01, range_m=2.1)
    request = decode(encode(resolve_update(decode(encode(stored)), 20)))
    applied = apply_resolve(stored, request, 6)
    assert applied is not None and applied.flags & FLAG_RESOLVED


def test_place_type_from_openings():
    from living_map_beacons.protocol import FLAG_CORRIDOR, FLAG_DEAD_END, FLAG_JUNCTION, place_flags
    assert [place_flags(n) for n in (0, 1, 2, 3, 4)] == [0, FLAG_DEAD_END, FLAG_CORRIDOR, FLAG_JUNCTION, FLAG_JUNCTION]
    packet = decode(encode(BeaconPacket(flags=FLAG_CORRIDOR | FLAG_STANDOFF, event_type=4)))
    assert roles(packet.flags) == ["STANDOFF", "CORRIDOR"]
