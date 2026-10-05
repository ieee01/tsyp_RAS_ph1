import math

from living_map_beacons.placement import PlacementConfig, PlacementPolicy, ring_openings
from living_map_beacons.protocol import FLAG_ENTRANCE, FLAG_JUNCTION, FLAG_RELAY, FLAG_SPACING

RES = 0.1
SIZE = 80  # 8 m x 8 m grid centred on (0, 0)
ORIGIN = (-4.0, -4.0)


def grid(passable):
    """Occupied everywhere except where ``passable(x, y)`` holds (free)."""
    data = []
    for row in range(SIZE):
        for column in range(SIZE):
            x, y = ORIGIN[0] + (column + .5) * RES, ORIGIN[1] + (row + .5) * RES
            data.append(0 if passable(x, y) else 100)
    return data


def openings(data):
    return ring_openings(data, SIZE, SIZE, RES, ORIGIN, (0.0, 0.0))


def test_corridor_has_two_openings_and_is_not_a_junction():
    assert openings(grid(lambda x, y: abs(y) < 0.8)) == 2


def test_t_junction_has_three_openings():
    assert openings(grid(lambda x, y: abs(y) < 0.8 or (y > 0 and abs(x) < 0.8))) == 3


def test_crossroads_has_four_openings():
    assert openings(grid(lambda x, y: abs(y) < 0.8 or abs(x) < 0.8)) == 4


def test_open_ground_and_dead_end():
    assert openings(grid(lambda x, y: True)) == 0
    assert openings(grid(lambda x, y: abs(y) < 0.8 and x < 0.5)) == 1


def test_unknown_space_counts_as_a_way_on():
    data = grid(lambda x, y: abs(y) < 0.8 or (y > 0 and abs(x) < 0.8))
    data = [-1 if value == 0 and index // SIZE > SIZE // 2 + 10 else value for index, value in enumerate(data)]
    assert openings(data) == 3


def test_narrow_cracks_are_not_openings():
    assert openings(grid(lambda x, y: abs(y) < 0.8 or (y > 0 and abs(x) < 0.1))) == 2


def test_policy_priorities_and_budget():
    policy = PlacementPolicy(PlacementConfig(capacity=10, event_reserve=4, spacing_m=6.0))
    assert policy.decide((0, 0), [], 0, None, 0, 0.0).flags == FLAG_ENTRANCE
    beacons = [(0.0, 0.0)]
    assert policy.decide((3, 0), beacons, 1, 0.98, 0, 1.0) is None
    assert policy.decide((3, 0), beacons, 1, 0.98, 3, 1.0).flags == FLAG_JUNCTION
    assert policy.decide((6.5, 0), beacons, 1, 0.98, 0, 1.0).flags == FLAG_SPACING
    relay = policy.decide((3, 0), beacons, 1, 0.5, 3, 1.0)
    assert relay.flags == FLAG_RELAY  # keeping the radio chain beats marking the junction
    assert policy.decide((4, 0), beacons, 1, 0.5, 0, 2.0) is None  # relay cooldown
    # With only the event reserve left, breadcrumbs and junctions stop; relays still allowed.
    assert policy.decide((6.5, 0), beacons, 6, 0.98, 3, 10.0) is None
    assert policy.decide((6.5, 0), beacons, 7, 0.4, 0, 10.0).flags == FLAG_RELAY
    assert policy.decide((9, 0), beacons, 9, 0.4, 0, 20.0) is None
    assert policy.can_deploy_event(9) and not policy.can_deploy_event(10)


def test_relay_needs_separation_from_existing_beacons():
    policy = PlacementPolicy()
    assert policy.decide((0.5, 0), [(0.0, 0.0)], 1, 0.4, 0, 0.0) is None
    assert math.isinf(policy.last_relay_at)
