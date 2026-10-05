import math

from living_map_writer.frontier import choose_frontier, clusters, frontier_cells
def test_frontier_found():
 w=h=7;d=[100]*(w*h)
 for y in range(2,5):
  for x in range(2,5):d[y*w+x]=0
 for x in range(1,6):d[1*w+x]=-1
 assert choose_frontier(d,w,h,1,0,0,3,3,min_cluster=2) is not None


def test_goal_is_free_cell_on_concave_frontier():
    w = h = 11
    data = [-1] * (w * h)
    for y in range(2, 9):
        for x in range(2, 9):
            if x <= 3 or y >= 7:
                data[y * w + x] = 0
    goal = choose_frontier(data, w, h, 1, 0, 0, 2.5, 7.5)
    assert goal is not None
    assert data[int(goal[2]) * w + int(goal[1])] == 0


def test_blacklisted_goal_does_not_stop_exploration():
    w = h = 11
    data = [-1] * (w * h)
    for y in range(2, 9):
        for x in range(2, 9):
            data[y * w + x] = 0
    first = choose_frontier(data, w, h, 1, 0, 0, 5.5, 5.5)
    second = choose_frontier(data, w, h, 1, 0, 0, 5.5, 5.5,
                             blacklist=[(first[1], first[2])])
    assert second is not None
    assert second[1:3] != first[1:3]


def test_unreachable_frontier_is_ignored():
    w = h = 11
    data = [100] * (w * h)
    data[5 * w + 2] = 0
    for y in range(2, 9):
        for x in range(6, 9):
            data[y * w + x] = 0
    for y in range(2, 9):
        data[y * w + 9] = -1
    assert choose_frontier(data, w, h, 1, 0, 0, 2.5, 5.5) is None


def test_small_unknown_patch_under_robot_does_not_block_exploration():
    w = h = 21
    data = [-1] * (w * h)
    for y in range(2, 19):
        for x in range(2, 19):
            data[y * w + x] = 0
    data[10 * w + 10] = -1
    goal = choose_frontier(data, w, h, 0.1, 0, 0, 1.05, 1.05)
    assert goal is not None
    assert data[int(goal[2] / 0.1) * w + int(goal[1] / 0.1)] == 0


def test_frontier_that_survives_a_visit_is_skipped():
    # Free room (x 2..17) with two frontiers: an open west edge (x=1 unknown)
    # that never resolves, and an east edge (x=18 unknown).
    w, h = 20, 9
    data = [100] * (w * h)
    for y in range(2, 7):
        data[y * w + 1] = -1
        data[y * w + 18] = -1
        for x in range(2, 18):
            data[y * w + x] = 0
    near_west = choose_frontier(data, w, h, 1, 0, 0, 3.5, 4.5, min_cluster=2, clearance=0.3)
    assert near_west[1] < 5
    # The robot reached that west goal, yet the frontier is still there.
    after_visit = choose_frontier(data, w, h, 1, 0, 0, 3.5, 4.5, min_cluster=2, clearance=0.3,
                                  visited=[(near_west[1], near_west[2])])
    assert after_visit is not None and after_visit[1] > 15


def test_visited_point_does_not_discard_connected_unexplored_boundary():
    """One visited section of a room perimeter must leave other goals available."""
    width = height = 21
    data = [-1] * (width * height)
    for y in range(2, 19):
        for x in range(2, 19):
            data[y * width + x] = 0
    assert len(clusters(frontier_cells(data, width, height))) == 1
    first = choose_frontier(data, width, height, 0.1, 0, 0, 1.05, 1.05)
    assert first is not None
    visited = [first[1:3]]
    second = choose_frontier(data, width, height, 0.1, 0, 0, *first[1:3],
                             visited=visited, visited_radius=0.5)
    assert second is not None
    assert math.dist(second[1:3], first[1:3]) >= 0.5
    assert data[int(second[2] / 0.1) * width + int(second[1] / 0.1)] == 0


def test_fully_visited_boundary_has_no_new_goal():
    """Suppress already visited cells without falling back to repeat goals."""
    width = height = 11
    data = [-1] * (width * height)
    for y in range(2, 9):
        for x in range(2, 9):
            data[y * width + x] = 0
    visited = [(x + 0.5, y + 0.5) for x, y in frontier_cells(data, width, height)]
    assert choose_frontier(data, width, height, 1, 0, 0, 5.5, 5.5,
                           visited=visited) is None


def test_frontiers_behind_the_entrance_are_not_explored():
    from living_map_writer.frontier import choose_frontier
    width, height = 40, 20
    # Free 1.2 m corridor with walls along its sides; unknown beyond both ends.
    data = [-1] * (width * height)
    for y in range(3, 17):
        for x in range(5, 35):
            data[y * width + x] = 0 if 4 <= y < 16 else 100  # walls along both sides
    # Robot near the west end, so the west frontier is closer and would win.
    west = choose_frontier(data, width, height, 0.1, 0.0, 0.0, 1.5, 1.0, min_cluster=2)
    assert west is not None and west[1] < 1.0
    bounded = choose_frontier(data, width, height, 0.1, 0.0, 0.0, 1.5, 1.0, min_cluster=2, min_x=1.0)
    assert bounded is not None and bounded[1] > 3.0
