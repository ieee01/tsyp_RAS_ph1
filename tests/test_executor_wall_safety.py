"""Prevent body collisions and shortcuts through inherited-map walls."""

from pathlib import Path
import math

import yaml

from living_map_executor.route import clear_spawn_footprint, footprint_is_free, nearest_clear_waypoint, shortcut_waypoints


def test_body_clearance_rejects_free_center_next_to_wall():
    """A free center cell alone does not make a safe goal for the Husky."""
    free = lambda x, y: x < 2
    assert free(1.5, 0)
    assert not footprint_is_free((1.5, 0), free, 1.1)
    goal = nearest_clear_waypoint((1.5, 0), lambda p: footprint_is_free(p, free, 1.1), 1.5, (0, 0))
    assert goal is not None
    assert footprint_is_free(goal, free, 1.1)
    assert math.dist(goal, (1.5, 0)) <= 1.5


def test_unreachable_goal_region_is_rejected():
    assert nearest_clear_waypoint((0, 0), lambda p: False, 1.6, (-2, 0)) is None


def test_shortcut_preserves_wall_detour():
    route = [(0, 3), (4, 3), (4, 0)]
    assert shortcut_waypoints((0, 0), route, [], 2.5, is_clear=lambda a, b: b == (0, 3)) == route


def test_executor_uses_inherited_map_and_consistent_collision_margin():
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / "config/nav2/executor.yaml").read_text())
    global_map = config["/executor/global_costmap/global_costmap"]["ros__parameters"]
    local_map = config["/executor/local_costmap/local_costmap"]["ros__parameters"]
    controller = config["/executor/controller_server"]["ros__parameters"]["FollowPath"]
    assert "static_layer" in global_map["plugins"]
    assert global_map["static_layer"]["map_topic"] == "/executor/navigation_map"
    assert global_map["track_unknown_space"]
    assert config["/executor/planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"]
    assert controller["use_collision_detection"]
    assert global_map["footprint"] == local_map["footprint"]
    assert controller["inflation_cost_scaling_factor"] == local_map["inflation_layer"]["cost_scaling_factor"]


def test_spawn_imprint_removed_without_changing_other_walls_or_source_map():
    data = [100] * 100
    cleaned = clear_spawn_footprint(data, 10, 10, 1, (0, 0), (2.5, 2.5), 1.1)
    assert cleaned[22] == 0
    assert cleaned[23] == 0
    assert cleaned[55] == 100
    assert data[22] == 100
