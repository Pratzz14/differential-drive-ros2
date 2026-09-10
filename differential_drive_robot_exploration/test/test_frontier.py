from differential_drive_robot_exploration.frontier import extract_frontiers, GridSpec


def test_frontier_components_are_clustered_and_safe():
    spec = GridSpec(7, 5, 0.1, 0.0, 0.0)
    data = [0] * (spec.width * spec.height)
    for y in range(spec.height):
        data[spec.index(3, y)] = 100
    for cell in ((1, 1), (1, 2), (2, 1), (2, 2)):
        data[spec.index(*cell)] = -1
    frontiers = extract_frontiers(data, spec, min_cells=2, robot_radius=0.1)
    assert len(frontiers) == 1
    assert frontiers[0].information_gain_m == 0.8
    assert frontiers[0].clearance_m >= 0.1


def test_small_unknown_boundary_is_filtered():
    spec = GridSpec(3, 3, 0.1, 0.0, 0.0)
    data = [0] * 9
    data[spec.index(1, 1)] = -1
    assert extract_frontiers(data, spec, min_cells=9) == []
