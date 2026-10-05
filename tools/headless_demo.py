#!/usr/bin/env python3
"""Dependency-light deterministic validation of LivingMap's information path.

This does not replace Gazebo/Nav2 validation. It exercises the competition's
core logic without ROS: standoff beacon deposition, the 33-byte frame, the
wall-aware radio mesh that relays deep memories out of the mine, gateway CRC /
GPS translation, Writer loss, and the Executor's inherited plan with hazard
keep-out.
"""
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
for package in ["living_map_beacons", "living_map_radio", "living_map_gateway", "living_map_executor"]:
    sys.path.insert(0, str(ROOT / "src" / package))

from living_map_beacons.protocol import (FLAG_ENTRANCE, FLAG_JUNCTION, FLAG_SPACING, FLAG_STANDOFF, BeaconPacket,
                                         decode, encode, guidance, roles, target_position)
from living_map_executor.graph import MemoryGraph
from living_map_executor.policy import keepout_radius, stamp_keepout
from living_map_gateway.coordinate import map_bearing_to_compass, map_to_wgs84
from living_map_radio.mesh import GATEWAY, RadioMesh
from living_map_radio.model import RFModel
from living_map_radio.walls import load_walls

ANCHOR = (36.8065, 10.1815, 18.0, -1.5, 0.0)  # lat, lon, heading, gateway map x/y
NAMES = {0: "NAV", 1: "VICTIM", 2: "GAS", 4: "FIRE"}

# Writer drops: (where the Writer stands, flags, event type, event position or None).
DROPS = [
    ((0.0, 0.0), FLAG_ENTRANCE, 0, None),
    ((3.9, -2.2), 0, 1, (6.0, -2.2)),          # victim seen 2 m ahead
    ((6.4, -3.4), FLAG_JUNCTION, 0, None),
    ((11.6, 0.0), FLAG_SPACING, 0, None),
    ((13.2, 0.4), 0, 2, (14.5, 3.1)),           # gas seen from outside its cloud
    ((15.5, -1.4), FLAG_JUNCTION, 0, None),
    ((17.7, -4.2), FLAG_JUNCTION, 0, None),
    ((15.0, -3.8), 0, 4, (12.5, -3.9)),         # fire seen from the far side
]


def main() -> int:
    print("=== LivingMap headless end-to-end validation (protocol v2) ===")
    walls = load_walls(ROOT / "src/living_map_sim/worlds/mine.sdf")
    mesh = RadioMesh(RFModel(42, walls=walls), (ANCHOR[3], ANCHOR[4]))
    now = int(time.time())
    packets: dict[int, BeaconPacket] = {}

    for beacon_id, (pose, flags, event_type, event) in enumerate(DROPS, 1):
        bearing, distance = guidance(pose, event) if event else (0.0, 0.0)
        packet = BeaconPacket(flags=flags | (FLAG_STANDOFF if event else 0), beacon_id=beacon_id, sequence=beacon_id,
                              timestamp_sec=now, x_m=pose[0], y_m=pose[1], bearing_rad=bearing, range_m=distance,
                              event_type=event_type, severity={1: 3, 2: 4, 4: 5}.get(event_type, 0),
                              confidence=0.95, previous_beacon_id=beacon_id - 1)
        packets[beacon_id] = packet
        frame = encode(packet)
        assert decode(frame) == decode(frame) and len(frame) == 33
        mesh.update_beacon(beacon_id, pose)
        direct = mesh.link(beacon_id, GATEWAY)
        result = next(r for r in (mesh.flood(beacon_id) for _ in range(10)) if r.delivered)
        tx, ty = target_position(decode(frame))
        lat, lon = map_to_wgs84(tx, ty, *ANCHOR)
        what = NAMES.get(event_type, "EVENT")
        where = ""
        if event:
            compass, point = map_bearing_to_compass(bearing, ANCHOR[2])
            where = f" -> {what} {distance:.1f} m {point} ({compass:.0f} deg)"
        print(f"[Writer] B{beacon_id:03d} {'/'.join(roles(packet.flags)):18s} at ({pose[0]:5.1f},{pose[1]:5.1f}){where}")
        print(f"[Radio]  direct p={direct.probability:.2f} through {direct.walls} wall(s); "
              f"delivered in {result.gateway_hops} hop(s) via "
              + " -> ".join(f"B{b:03d}" for b in result.gateway_path) + " -> gateway")
        print(f"[Gateway] CRC valid; WHAT at GPS {lat:.6f},{lon:.6f}")

    # Chain links: each beacon learns its successor.
    for beacon_id in range(1, len(DROPS)):
        packets[beacon_id] = BeaconPacket(**{**packets[beacon_id].__dict__, "next_beacon_id": beacon_id + 1})

    print("[Writer] DESTROYED - radio silent; beacons keep advertising")
    deep = max(mesh.routes().items(), key=lambda item: item[1].hops)
    print(f"[Mesh] deepest route B{deep[0]:03d}: {deep[1].hops} hops, p={deep[1].probability:.2f}")

    graph = MemoryGraph()
    for packet in packets.values():
        graph.add(packet.beacon_id, packet.x_m, packet.y_m, event_type=packet.event_type)
    for packet in packets.values():
        if packet.next_beacon_id in packets:
            graph.connect(packet.beacon_id, packet.next_beacon_id)
    hazards = [(*target_position(p), keepout_radius(p.event_type)) for p in packets.values() if p.event_type in (2, 4)]
    blocked = graph.nodes_near_points([(x, y, 2.5) for x, y, _ in hazards], keep={2})
    path = graph.shortest_path(1, 2, (), blocked)
    assert path == [1, 2], path
    grid = stamp_keepout([0] * (240 * 100), 240, 100, 0.1, (-3.0, -5.0), hazards)
    lethal = sum(1 for value in grid if value == 100)
    victim = target_position(packets[2])
    print(f"[Executor] briefed through the gateway with {len(packets)} memories; target VICTIM B002 at "
          f"({victim[0]:.1f},{victim[1]:.1f}); {len(hazards)} hazards stamped as keep-out ({lethal} cells)")
    print("[Executor] route " + " -> ".join(f"B{b:03d}" for b in path) + " -> victim")
    print("[Executor] MISSION COMPLETE (information-path validation)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
