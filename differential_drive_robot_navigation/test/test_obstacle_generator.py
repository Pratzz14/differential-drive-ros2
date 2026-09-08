import math

from differential_drive_robot_navigation.obstacle_generator import (
    GOAL,
    MAX_OBSTACLES,
    RESERVED_RADIUS,
    START,
    Obstacle,
    generate_layout,
    route_exists,
)


def test_seed_is_reproducible():
    assert generate_layout(42) == generate_layout(42)


def test_different_seeds_change_layout():
    assert generate_layout(42).obstacles != generate_layout(43).obstacles


def test_generated_layouts_are_traversable():
    for seed in range(20):
        assert route_exists(generate_layout(seed).obstacles)


def test_start_and_goal_are_clear():
    for obstacle in generate_layout(7, MAX_OBSTACLES).obstacles:
        for pose in (START, GOAL):
            nearest_x = max(
                obstacle.x - obstacle.width / 2.0,
                min(pose[0], obstacle.x + obstacle.width / 2.0),
            )
            nearest_y = max(
                obstacle.y - obstacle.depth / 2.0,
                min(pose[1], obstacle.y + obstacle.depth / 2.0),
            )
            assert math.hypot(
                pose[0] - nearest_x, pose[1] - nearest_y
            ) >= RESERVED_RADIUS


def test_blocking_wall_has_no_route():
    wall = Obstacle(0.0, 0.0, 0.4, 8.0, 'wall', 1.0, 0.0, 0.0)
    assert not route_exists((wall,))


def test_count_range_is_validated():
    try:
        generate_layout(1, MAX_OBSTACLES + 1)
    except ValueError:
        pass
    else:
        raise AssertionError('Expected invalid obstacle count to raise ValueError')


def test_custom_automatic_goal_is_clear_and_reachable():
    custom_goal = (3.0, -1.5)
    layout = generate_layout(42, MAX_OBSTACLES, goal=custom_goal)
    assert route_exists(layout.obstacles, start=START, goal=custom_goal)
    for obstacle in layout.obstacles:
        nearest_x = max(
            obstacle.x - obstacle.width / 2.0,
            min(custom_goal[0], obstacle.x + obstacle.width / 2.0),
        )
        nearest_y = max(
            obstacle.y - obstacle.depth / 2.0,
            min(custom_goal[1], obstacle.y + obstacle.depth / 2.0),
        )
        assert math.hypot(
            custom_goal[0] - nearest_x, custom_goal[1] - nearest_y
        ) >= RESERVED_RADIUS


def test_goal_outside_arena_is_rejected():
    try:
        generate_layout(1, goal=(100.0, 0.0))
    except ValueError:
        pass
    else:
        raise AssertionError('Expected an out-of-bounds goal to raise ValueError')
