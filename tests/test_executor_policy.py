import pytest

from living_map_executor.graph import MemoryGraph
from living_map_executor.policy import (DEFAULT_AVOID_TYPES, avoid_types, edge_cost, keepout_radius,
                                        stamp_keepout)


def test_robot_keeps_its_own_hazard_policy():
    assert avoid_types([]) == DEFAULT_AVOID_TYPES == {2, 3, 4}
    assert avoid_types([2]) == {2, 3, 4}  # a brief can add, never remove
    assert keepout_radius(2) > keepout_radius(3)


def test_keepout_disc_is_lethal_except_the_protected_target():
    data = [0] * 100  # 10 x 10 cells of 1 m
    stamped = stamp_keepout(data, 10, 10, 1.0, (0.0, 0.0), [(5.0, 5.0, 2.0)], [(6.5, 5.5, 0.6)])
    assert stamped[5 * 10 + 5] == 100
    assert stamped[5 * 10 + 6] == 0  # protected approach cell
    assert stamped[0] == 0 and stamped[9 * 10 + 9] == 0
    assert data == [0] * 100


def test_driven_links_are_cheaper_than_inferred_and_age_costs_more():
    assert edge_cost(4.0, True, "FRESH", "FRESH") == pytest.approx(4.0)
    assert edge_cost(4.0, False, "FRESH", "FRESH") > edge_cost(4.0, True, "FRESH", "FRESH")
    assert edge_cost(4.0, True, "STALE", "FRESH") > edge_cost(4.0, True, "AGING", "FRESH")


def test_inferred_edges_never_overwrite_driven_edges():
    graph = MemoryGraph()
    for node_id, x in ((1, 0.0), (2, 3.0)):
        graph.add(node_id, x, 0.0)
    graph.connect(1, 2, 3.0)
    graph.connect_spatial_neighbors(5.0, cost=lambda a, b, d: d * 10)
    assert graph.edges[1][2] == 3.0


def test_nodes_inside_hazard_discs_are_blocked_except_kept():
    graph = MemoryGraph()
    graph.add(1, 0.0, 0.0)
    graph.add(2, 1.0, 0.0)
    graph.add(3, 5.0, 0.0)
    assert graph.nodes_near_points([(0.5, 0.0, 1.0)], keep={2}) == {1}


def test_path_cost_sums_edges():
    graph = MemoryGraph()
    for node_id, x in ((1, 0.0), (2, 3.0), (3, 7.0)):
        graph.add(node_id, x, 0.0)
    graph.connect(1, 2)
    graph.connect(2, 3, 10.0)
    assert graph.path_cost([1, 2, 3]) == 13.0
    assert graph.path_cost([2]) == 0.0
