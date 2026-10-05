import json
from pathlib import Path

import pytest

from living_map_radio.mesh import GATEWAY, RadioMesh
from living_map_radio.model import RFModel, walls_crossed
from living_map_radio.walls import load_walls

ROOT = Path(__file__).resolve().parents[1]
WALLS = load_walls(ROOT / "src/living_map_sim/worlds/mine.sdf")
GATEWAY_XY = (-1.5, 0.0)


def mesh(seed=7, **kwargs):
    return RadioMesh(RFModel(seed, walls=WALLS), GATEWAY_XY, **kwargs)


def test_world_walls_are_loaded_in_the_writer_map_frame():
    # wall3 sits at world x=-8, i.e. map x=3 once the Writer spawn (-11, 0) is the origin.
    assert (3.0, 3.0, 0.35, 4.0) in WALLS
    assert len(WALLS) == 9


def test_rock_walls_attenuate_the_link():
    model = RFModel(walls=WALLS)
    open_drift = model.link(GATEWAY_XY, (10.0, -4.0))
    behind_rock = model.link(GATEWAY_XY, (14.5, 3.1))
    assert open_drift.walls == 0 and open_drift.probability > 0.9
    assert behind_rock.walls == 2 and behind_rock.probability == 0.0
    assert behind_rock.rssi_dbm < open_drift.rssi_dbm - 15


def test_segment_wall_crossing_counts_each_wall_once():
    assert walls_crossed((0.0, 0.0), (6.0, 0.0), [(3.0, 0.0, 0.35, 4.0)]) == 1
    assert walls_crossed((0.0, 3.0), (6.0, 3.0), [(3.0, 0.0, 0.35, 4.0)]) == 0


def test_deep_memory_reaches_the_gateway_only_through_relays():
    radio = mesh()
    deep = 9
    for beacon_id, point in {1: (0.0, 0.0), 2: (6.0, -2.0), 3: (12.0, -2.0), 4: (15.0, -1.8),
                             5: (15.4, 0.5), deep: (14.0, 3.0)}.items():
        radio.update_beacon(beacon_id, point)
    assert radio.link(deep, GATEWAY).probability == 0.0
    routes = radio.routes()
    assert routes[deep].hops >= 3
    assert routes[deep].path[0] == deep
    delivered = [radio.flood(deep) for _ in range(20)]
    assert sum(result.delivered for result in delivered) >= 18
    assert all(result.gateway_hops >= 2 for result in delivered if result.delivered)


def test_destroyed_relay_is_routed_around_or_reported_unreachable():
    radio = mesh()
    for beacon_id, point in {1: (6.0, -2.0), 2: (12.0, -2.0), 3: (15.4, -1.0), 4: (15.4, 0.8),
                             5: (14.0, 3.0), 6: (12.5, -4.0)}.items():
        radio.update_beacon(beacon_id, point)
    before = radio.routes()[5]
    relay = before.path[1]
    radio.kill(relay)
    after = radio.routes().get(5)
    assert relay not in radio.alive()
    if after is not None:
        assert relay not in after.path
    else:
        assert 5 in radio.topology()["unreachable"]
    assert not radio.flood(relay).delivered


def test_robot_telemetry_uses_the_best_route():
    radio = mesh()
    radio.update_beacon(1, (6.0, -2.0))
    radio.update_beacon(2, (12.0, -2.0))
    radio.update_beacon(3, (15.4, 0.0))
    routes = radio.routes()
    near = radio.route_from((0.0, 0.0), routes)
    assert near is not None and near.hops == 1 and near.path == ()
    deep = radio.route_from((14.5, 3.0), routes)
    assert deep is not None and deep.hops >= 2
    assert radio.route_from((19.5, 4.5), {}) is None


def test_uplink_quality_is_what_the_writer_measures():
    radio = mesh()
    radio.update_beacon(1, (6.0, -2.0))
    routes = radio.routes()
    assert radio.uplink_quality((7.0, -2.0), routes) > 0.95
    assert radio.uplink_quality((14.5, 3.1), routes) < 0.5


def test_mesh_sampling_is_reproducible():
    def run():
        radio = mesh(seed=3)
        for beacon_id, point in {1: (6.0, -2.0), 2: (12.0, -2.0), 3: (15.4, 0.0)}.items():
            radio.update_beacon(beacon_id, point)
        return [radio.flood(3).gateway_hops for _ in range(10)]
    assert run() == run()


def test_topology_snapshot_is_serialisable():
    radio = mesh()
    radio.update_beacon(1, (6.0, -2.0))
    radio.update_beacon(2, (14.0, 3.0))
    topology = radio.topology({"writer": (13.0, 2.5)})
    json.dumps(topology)
    assert topology["routes"]["1"]["hops"] == 1
    assert topology["robots"]["writer"]["connected"] is False
    assert topology["robots"]["writer"]["uplink_p"] == pytest.approx(
        radio.uplink_quality((13.0, 2.5)), abs=1e-3)
