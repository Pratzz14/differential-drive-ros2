from pathlib import Path
# flake8: noqa

from differential_drive_robot_exploration.truth import (
    Alignment,
    Box,
    freeze_alignment,
    parse_static_boxes,
    Pose2D,
    score_grid,
)
from differential_drive_robot_exploration.frontier import GridSpec


ROOT = Path(__file__).parents[2]


def test_static_world_contains_walls_and_divider():
    boxes = parse_static_boxes(ROOT / 'differential_drive_robot_simulation/worlds/navigation_arena.sdf.in')
    names = {box.name.split('/')[0] for box in boxes}
    assert {'north_wall', 'south_wall', 'east_wall', 'west_wall', 'central_divider'} <= names


def test_alignment_maps_initial_robot_pose_to_truth():
    alignment = freeze_alignment(Pose2D(1.0, 0.0, 0.0), Pose2D(4.0, 2.0, 0.0))
    assert alignment.map_to_world(1.0, 0.0) == (4.0, 2.0)


def test_coverage_denominator_includes_truth_beyond_current_map():
    boxes = (
        Box(2.0, -0.05, 4.2, 0.1),
        Box(2.0, 4.05, 4.2, 0.1),
        Box(-0.05, 2.0, 0.1, 4.2),
        Box(4.05, 2.0, 0.1, 4.2),
    )
    spec = GridSpec(2, 2, 1.0, 0.0, 0.0)
    result = score_grid(
        [0, 0, 0, 0], spec, boxes, Alignment(0.0, 0.0, 0.0),
        (0.5, 0.5), robot_radius=0.2,
    )
    assert result['reachable_free_cells'] > len([0, 0, 0, 0])
    assert result['coverage_percent'] < 100.0
    assert result['map_accuracy_percent'] <= 50.0
