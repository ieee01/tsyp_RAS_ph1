import math

from living_map_executor.route import (approach_from_first_in_reach, hazard_avoiding_waypoints,
                                       segment_distance)


def test_hazard_detour_inserted():
    waypoints = hazard_avoiding_waypoints((0, 0), (10, 0), [(5, 0.2)], 1.0)
    assert len(waypoints) == 2
    assert waypoints[-1] == (10, 0)
    assert segment_distance(5, 0.2, 0, 0, *waypoints[0]) > 0


def test_blocked_first_side_uses_opposite_side():
    def only_negative_side_is_free(_x, y):
        return y < 0

    waypoints = hazard_avoiding_waypoints(
        (0, 0), (10, 0), [(5, 0)], 1.0, is_free=only_negative_side_is_free
    )
    assert waypoints[0][1] < 0


def test_both_sides_blocked_at_first_offset_grows_offset():
    def farther_points_are_free(_x, y):
        return abs(y) > 2.3

    waypoints = hazard_avoiding_waypoints(
        (0, 0), (10, 0), [(5, 0)], 1.0, is_free=farther_points_are_free
    )
    assert abs(waypoints[0][1]) == 2.8


def test_multiple_hazards_are_ordered_along_direct_path():
    waypoints = hazard_avoiding_waypoints(
        (0, 0), (10, 0), [(8, 0), (2, 0), (5, 0)], 1.0
    )
    assert [point[0] for point in waypoints[:-1]] == [2, 5, 8]


def test_segment_is_clear_detects_blocked_sample():
    from living_map_executor.route import segment_is_clear

    wall = lambda x, y: 1.9 <= x <= 2.1
    assert not segment_is_clear((0.0, 0.0), (4.0, 0.0), wall, 0.05)
    assert segment_is_clear((0.0, 0.0), (1.5, 1.0), wall, 0.05)


def test_segment_is_clear_ignores_objects_at_its_endpoints():
    from living_map_executor.route import segment_is_clear

    # A mapped victim of radius 0.4 m centred on the segment's end point.
    victim = lambda x, y: (x - 4.0) ** 2 + y ** 2 <= 0.4 ** 2
    assert not segment_is_clear((0.0, 0.0), (4.0, 0.0), victim, 0.05)
    assert segment_is_clear((0.0, 0.0), (4.0, 0.0), victim, 0.05, end_margin=0.6)
    # A wall in the middle of the edge is still detected.
    wall = lambda x, y: 1.9 <= x <= 2.1
    assert not segment_is_clear((0.0, 0.0), (4.0, 0.0), wall, 0.05, end_margin=0.6)


def test_shortcut_skips_waypoints_that_do_not_avoid_a_hazard():
    from living_map_executor.route import shortcut_waypoints

    pose = (0.0, 0.0)
    route = [(-1.0, 2.0), (4.0, 0.0), (8.0, 0.0)]
    # No hazards: go straight for the last waypoint.
    assert shortcut_waypoints(pose, route, [], 2.5) == [(8.0, 0.0)]
    # A hazard on the direct line keeps the detour waypoint that avoids it.
    detour = [(4.0, 4.0), (8.0, 0.0)]
    assert shortcut_waypoints(pose, detour, [(4.0, 0.0)], 2.5) == detour


def test_fire_approach_replaces_the_standoff_beacon_with_a_straight_stop():
    # 5 October run: the fire robot was routed through standoff beacon B002, 2.67 m from
    # the fire, then on to a second stop, and stalled turning in place at B002.
    fire = (8.0, -2.0)
    route, labels = approach_from_first_in_reach(
        [(5.48, -2.89), (5.78, -1.08)], ["B002", "FIRE B002"], fire, reach=3.4, stop=2.45)
    assert labels == ["FIRE B002"]
    (x, y), = route
    assert math.isclose(math.dist((x, y), fire), 2.45, abs_tol=1e-9)
    # On the line from the beacon to the fire, so the robot drives straight in.
    assert math.isclose(math.atan2(fire[1] - y, fire[0] - x), math.atan2(fire[1] + 2.89, fire[0] - 5.48))
    # Even stopping 0.25 m short of the goal (Nav2 tolerance) is within the 2.8 m arrival distance.
    assert math.dist((x, y), fire) + 0.25 < 2.8


def test_earlier_waypoints_are_kept():
    route = [(0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (7.0, 1.0)]
    labels = ["B001", "B002", "B003", "FIRE"]
    new_route, new_labels = approach_from_first_in_reach(route, labels, (9.0, 0.0), reach=3.4, stop=2.45)
    assert new_route[:2] == [(0.0, 0.0), (4.0, 0.0)] and math.isclose(new_route[2][0], 6.55)
    assert new_labels == ["B001", "B002", "FIRE"]


def test_waypoint_already_close_enough_becomes_the_stop():
    route, labels = approach_from_first_in_reach([(6.0, 0.0), (7.0, 1.0)], ["B003", "FIRE"], (8.0, 0.0),
                                                 reach=3.4, stop=2.45)
    assert (route, labels) == ([(6.0, 0.0)], ["B003"])


def test_route_is_unchanged_when_no_waypoint_is_within_reach():
    route = [(0.0, 0.0), (4.0, 0.0), (7.0, 0.0)]
    labels = ["B001", "B002", "FIRE B003"]
    assert approach_from_first_in_reach(route, labels, (12.0, 0.0), reach=3.4, stop=2.45) == (route, labels)
