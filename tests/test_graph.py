from living_map_executor.graph import MemoryGraph


def test_shortest_path_avoids_hazard_type():
    graph = MemoryGraph()
    graph.add(1, 0, 0, event_type=0)
    graph.add(2, 1, 0, event_type=2)
    graph.add(3, 0, 1, event_type=0)
    graph.add(4, 1, 1, event_type=1)
    graph.connect(1, 2, 1)
    graph.connect(2, 4, 1)
    graph.connect(1, 3, 1.2)
    graph.connect(3, 4, 1.2)
    assert graph.shortest_path(1, 4, blocked_types=(2,)) == [1, 3, 4]


def test_hazard_radius_blocks_nearby_nav_and_spatial_edges_preserve_route():
    graph = MemoryGraph()
    graph.add(1, 0.0, 0.0, event_type=0)
    graph.add(2, 3.0, 0.0, event_type=0)
    graph.add(3, 3.0, 1.0, event_type=2)
    graph.add(4, 2.0, 4.0, event_type=0)
    graph.add(5, 6.0, 4.0, event_type=1)

    blocked = graph.nodes_near_hazards({3}, radius_m=2.5, target_id=5)
    assert {2, 3} <= blocked
    graph.connect_spatial_neighbors(4.5, blocked)

    path = graph.shortest_path(1, 5, blocked_node_ids=blocked)
    assert path == [1, 4, 5]
    assert 2 not in path
    assert 3 not in path


def test_target_is_never_blocked_by_hazard_radius():
    graph = MemoryGraph()
    graph.add(1, 0.0, 0.0, event_type=2)
    graph.add(2, 0.5, 0.0, event_type=1)
    blocked = graph.nodes_near_hazards({1}, radius_m=2.5, target_id=2)
    assert 1 in blocked
    assert 2 not in blocked

    graph.connect(1, 2)
    assert graph.shortest_path(1, 2, blocked_types={1}, blocked_node_ids={1}) == []
    assert graph.shortest_path(2, 1, blocked_types={2}, blocked_node_ids={1}) == [2, 1]


def test_spatial_edges_respect_edge_predicate():
    graph = MemoryGraph()
    graph.add(1, 0.0, 0.0, event_type=0)
    graph.add(2, 3.0, 0.0, event_type=0)
    graph.add(3, 1.5, 2.0, event_type=0)
    wall_between_1_and_2 = {frozenset((1, 2))}
    graph.connect_spatial_neighbors(
        4.5, edge_ok=lambda a, b: frozenset((a, b)) not in wall_between_1_and_2
    )
    assert 2 not in graph.edges[1]
    assert graph.shortest_path(1, 2) == [1, 3, 2]


def test_shortest_path_rejects_edges_passing_through_hazard_radius():
    from living_map_executor.route import segment_distance

    graph = MemoryGraph()
    graph.add(1, 0.0, 0.0, event_type=0)
    graph.add(2, 4.0, 0.0, event_type=0)
    graph.add(3, 2.0, 3.5, event_type=0)
    graph.add(4, 6.0, 0.0, event_type=1)
    graph.connect(1, 2)
    graph.connect(1, 3)
    graph.connect(3, 2)
    graph.connect(2, 4)
    hazard = (2.0, 0.5)

    def clear(a, b):
        first, second = graph.nodes[a], graph.nodes[b]
        return segment_distance(*hazard, first["x"], first["y"], second["x"], second["y"]) > 1.0

    assert graph.shortest_path(1, 4) == [1, 2, 4]
    assert graph.shortest_path(1, 4, edge_ok=clear) == [1, 3, 2, 4]
