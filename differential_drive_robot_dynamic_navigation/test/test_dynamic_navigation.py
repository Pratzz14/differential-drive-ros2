from pathlib import Path

from differential_drive_robot_dynamic_navigation.dynamic_obstacle_controller import (
    _velocity_components, _body_velocity,
)
import pytest
import yaml
from differential_drive_robot_dynamic_navigation.motion_tracker import Track, _active_tracks, _update_velocity, _near_static_map
from nav_msgs.msg import OccupancyGrid
from differential_drive_robot_dynamic_navigation.obstacle_generator import (
    START, GOAL, Rect, _clear_pose, generate_layout, generate_world,
)


def test_seeded_layout_is_reproducible():
    assert generate_layout(42) == generate_layout(42)
    assert generate_layout(42) != generate_layout(43)


def test_dynamic_obstacles_have_safe_speed_and_clear_poses():
    layout = generate_layout(7, static_count=2, dynamic_count=4)
    assert all(0.10 <= obstacle.speed <= 0.22 for obstacle in layout.dynamic_obstacles)
    assert all(abs(obstacle.x - START[0]) > 0.2 for obstacle in layout.dynamic_obstacles)
    assert all(abs(obstacle.x - GOAL[0]) > 0.2 for obstacle in layout.dynamic_obstacles)


def test_world_render_contains_physical_patrol_models(tmp_path: Path):
    template = tmp_path / 'arena.sdf.in'
    template.write_text('<!-- STATIC_OBSTACLES --><!-- DYNAMIC_OBSTACLES -->', encoding='utf-8')
    _, world, metadata = generate_world(template, tmp_path, 2, 1, 1)
    content = world.read_text(encoding='utf-8')
    assert '<static>false</static>' in content
    assert 'gz-sim-velocity-control-system' in content
    assert metadata.is_file()


def test_vertical_patrol_commands_y_velocity():
    assert _velocity_components(0.0, -1.0, 0.12) == (0.0, -0.12)
    assert _velocity_components(0.0, -1.0, -0.12) == (-0.0, 0.12)


def test_displaced_rotated_patrol_steers_toward_endpoint():
    import math
    assert _body_velocity(1, 0, 0.22, math.pi / 2) == pytest.approx((0, -0.22))
    assert _body_velocity(-1, 0, 0.22, 0) == pytest.approx((-0.22, 0))
    assert _body_velocity(0, 0, 0.22, 0) == (0, 0)


def test_unmatched_tracks_expire():
    stale = Track(0, 1.0, 2.0, last_time=1.0)
    current = Track(1, 2.0, 3.0, last_time=1.8)
    assert _active_tracks([stale, current], stamp=2.0, timeout=0.5) == [current]


def test_clock_reset_discards_future_tracks():
    assert _active_tracks([Track(0, 0, 0, last_time=10.0)], 1.0, 0.5) == []


@pytest.mark.parametrize('seed', [7, 11, 42, 91, 123])
def test_entire_patrol_stays_clear_of_start_and_goal(seed):
    for obstacle in generate_layout(seed).dynamic_obstacles:
        corridor = Rect((obstacle.x + obstacle.end_x) / 2,
                        (obstacle.y + obstacle.end_y) / 2,
                        abs(obstacle.x - obstacle.end_x) + obstacle.width,
                        abs(obstacle.y - obstacle.end_y) + obstacle.depth)
        assert all(_clear_pose(corridor, pose, 0.85) for pose in (START, GOAL))


def test_velocity_fit_matches_slow_patrol():
    track = Track(0, 0, 0)
    for i in range(11):
        _update_velocity(track, i * 0.1, i * 0.01, -i * 0.005)
    assert track.vx == pytest.approx(0.1)
    assert track.vy == pytest.approx(-0.05)


def test_velocity_waits_for_observation_window():
    track = Track(0, 0, 0)
    _update_velocity(track, 0.0, 0.0, 0.0)
    _update_velocity(track, 0.1, 0.1, 0.1)
    assert (track.vx, track.vy) == (0.0, 0.0)


def test_partial_wall_cluster_is_not_a_moving_obstacle():
    grid = OccupancyGrid()
    grid.info.resolution = 0.1
    grid.info.width, grid.info.height = 20, 40
    grid.info.origin.position.x = -1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 800
    for row in range(20):
        grid.data[row * 20 + 10] = 100
    # Both a side fragment and the tip of the divider are static, even if
    # their visible centroid slides as the robot turns or occlusion changes.
    assert _near_static_map(0.05, 1.0, grid)
    assert _near_static_map(0.05, 2.12, grid)
    assert not _near_static_map(0.60, 2.5, grid)


def test_unknown_map_cells_do_not_hide_real_obstacles():
    grid = OccupancyGrid()
    grid.info.resolution = 0.1
    grid.info.width = grid.info.height = 10
    grid.info.origin.orientation.w = 1.0
    grid.data = [-1] * 100
    assert not _near_static_map(0.5, 0.5, grid)


@pytest.mark.parametrize('minimum,maximum', [(0.0, 0.2), (0.2, 0.1), (0.1, 0.31), (0.1, float('nan'))])
def test_rejects_invalid_patrol_speeds(minimum, maximum):
    with pytest.raises(ValueError):
        generate_layout(42, min_speed=minimum, max_speed=maximum)


def test_speed_override_preserves_seeded_geometry():
    normal = generate_layout(42)
    slower = generate_layout(42, min_speed=0.05, max_speed=0.15)
    for a, b in zip(normal.dynamic_obstacles, slower.dynamic_obstacles):
        assert (a.x, a.y, a.end_x, a.end_y) == (b.x, b.y, b.end_x, b.end_y)
        assert a.speed > b.speed


def test_global_plan_contains_robot_at_any_heading():
    import math
    config = yaml.safe_load((Path(__file__).parents[1] / 'config' / 'nav2.yaml').read_text())
    global_map = config['global_costmap']['global_costmap']['ros__parameters']
    local_map = config['local_costmap']['local_costmap']['ros__parameters']
    padding = local_map['footprint_padding']
    radius = global_map['robot_radius'] + global_map['footprint_padding']
    for x, y in yaml.safe_load(local_map['footprint']):
        assert radius >= math.hypot(abs(x) + padding, abs(y) + padding)


def test_start_occupied_has_bounded_bt_recovery():
    import xml.etree.ElementTree as ET
    from nav2_msgs.action import ComputePathToPose
    tree = ET.parse(Path(__file__).parents[1] / 'behavior_trees' / 'navigate.xml')
    conditions = [item.attrib['code'] for item in tree.iter('ScriptCondition')]
    assert conditions.count(f'compute_path_error_code == {ComputePathToPose.Result.START_OCCUPIED}') == 2
    main_recovery = next(item for item in tree.iter('RecoveryNode') if item.attrib.get('name') == 'NavigateRecovery')
    assert int(main_recovery.attrib['number_of_retries']) == 6


def test_rviz_has_robot_and_ros2_topic_qos():
    path = Path(__file__).parents[1] / 'config' / 'navigation.rviz'
    displays = {d['Class'].split('/')[-1]: d for d in
                yaml.safe_load(path.read_text())['Visualization Manager']['Displays']}
    robot = displays['RobotModel']['Description Topic']
    assert robot['Value'] == '/robot_description'
    assert robot['Durability Policy'] == 'Transient Local'
    scan = displays['LaserScan']['Topic']
    assert scan['Value'] == '/scan'
    assert scan['Reliability Policy'] == 'Best Effort'
    assert displays['MarkerArray']['Topic']['Value'] == '/dynamic_obstacles/tracks'
    maps = [d for d in yaml.safe_load(path.read_text())['Visualization Manager']['Displays']
            if d['Class'] == 'rviz_default_plugins/Map']
    assert any(d['Topic']['Value'] == '/map' and
               d['Topic']['Durability Policy'] == 'Transient Local' for d in maps)


def test_direct_route_tuning_preserves_local_prediction_and_safety():
    config = yaml.safe_load((Path(__file__).parents[1] / 'config' / 'nav2.yaml').read_text())
    global_map = config['global_costmap']['global_costmap']['ros__parameters']
    local_map = config['local_costmap']['local_costmap']['ros__parameters']
    controller = config['controller_server']['ros__parameters']['FollowPath']
    assert 'predicted_obstacle_layer' not in global_map['plugins']
    assert 'obstacle_layer' in global_map['plugins']
    assert 'predicted_obstacle_layer' in local_map['plugins']
    assert local_map['predicted_obstacle_layer']['enabled']
    assert controller['use_collision_detection']
    assert controller['use_cost_regulated_linear_velocity_scaling']
    assert controller['inflation_cost_scaling_factor'] == local_map['inflation_layer']['cost_scaling_factor']
    assert controller['cost_scaling_dist'] <= local_map['inflation_layer']['inflation_radius']
    assert global_map['inflation_layer']['inflation_radius'] >= global_map['robot_radius'] + global_map['footprint_padding']
    assert config['planner_server']['ros__parameters']['GridBased']['cost_travel_multiplier'] == 2.0


def test_bt_keeps_valid_paths_but_rechecks_blockages_and_new_goals():
    import xml.etree.ElementTree as ET
    tree = ET.parse(Path(__file__).parents[1] / 'behavior_trees' / 'navigate.xml')
    rate = tree.find('.//RateController')
    assert rate.attrib['hz'] == '2.0'
    fallback = rate.find('./RecoveryNode/Fallback')
    assert [child.tag for child in fallback] == ['ReactiveSequence', 'ComputePathToPose']
    reuse = fallback.find('ReactiveSequence')
    assert reuse.find('./Inverter/PathExpiringTimer').attrib['seconds'] == '10.0'
    assert reuse.find('./Inverter/GlobalUpdatedGoal') is not None
    validation = reuse.find('Fallback')
    assert validation[0].tag == 'IsPathValid'
    assert [child.tag for child in validation[1]] == ['Wait', 'IsPathValid']
    assert validation[1][0].attrib['wait_duration'] == '1.0'


def test_terminal_isolated_children_still_receive_launch_sigint(monkeypatch):
    from launch import LaunchContext
    from launch.actions import ExecuteProcess
    from differential_drive_robot_dynamic_navigation.session_process import SessionProcess
    received = []
    monkeypatch.setattr(ExecuteProcess, '_shutdown_process',
                        lambda self, context, *, send_sigint: received.append(send_sigint))
    process = SessionProcess(cmd=['true'])
    process._shutdown_process(LaunchContext(), send_sigint=False)
    assert received == [True]
    assert process.process_description.prefix[0].text == 'setsid'


@pytest.mark.parametrize('status,is_error', [
    ('canceled', False), ('succeeded', False), ('aborted', True), ('timed_out', True),
])
def test_expected_goal_cancellation_is_not_reported_as_a_crash(status, is_error):
    from types import SimpleNamespace
    from unittest.mock import MagicMock
    from differential_drive_robot_dynamic_navigation.mission_runner import MissionRunner
    logger = MagicMock()
    MissionRunner._report_result(SimpleNamespace(get_logger=lambda: logger), status, '/tmp/trial.json')
    assert logger.error.called == is_error
    if status == 'canceled':
        assert 'canceled' in logger.info.call_args_list[0].args[0]


@pytest.mark.parametrize('available,success,partition,pending_goal', [
    (True, True, 'differential_drive_test', False),
    (False, False, 'differential_drive_test', False),
    (True, False, 'another_users_simulator', False),
    (True, True, 'differential_drive_test', True),
])
def test_orderly_shutdown_is_bounded_and_partition_scoped(monkeypatch, available, success, partition, pending_goal):
    from itertools import count
    from types import SimpleNamespace
    from unittest.mock import MagicMock
    from nav2_msgs.srv import ManageLifecycleNodes
    from differential_drive_robot_dynamic_navigation import shutdown
    calls = []
    node = MagicMock()
    context = MagicMock()
    executor = MagicMock()
    goal = SimpleNamespace(goal_id=SimpleNamespace(uuid=[1] * 16))

    def client(_service_type, name):
        instance = MagicMock()
        instance.wait_for_service.return_value = available
        instance.call_async.side_effect = lambda request: (
            calls.append((name, getattr(request, 'command', None))) or
            SimpleNamespace(done=lambda: True, exception=lambda: None,
                            result=lambda: SimpleNamespace(success=success, return_code=0 if success else 1,
                                                           goals_canceling=[goal] if pending_goal else [])))
        return instance

    node.create_client.side_effect = client
    monkeypatch.setattr(shutdown, 'Context', lambda: context)
    monkeypatch.setattr(shutdown.rclpy, 'init', MagicMock())
    monkeypatch.setattr(shutdown.rclpy, 'create_node', lambda *a, **kw: node)
    monkeypatch.setattr(shutdown, 'SingleThreadedExecutor', lambda **kw: executor)
    ticks = count(0.0, 0.1)
    monkeypatch.setattr(shutdown.time, 'monotonic', lambda: next(ticks))
    if pending_goal:
        def finish_cancellation(**_kwargs):
            callback = node.create_subscription.call_args.args[2]
            callback(SimpleNamespace(status_list=[SimpleNamespace(goal_info=goal, status=5)]))
        executor.spin_once.side_effect = finish_cancellation
    gazebocall = MagicMock(return_value=SimpleNamespace(returncode=0, stdout='data: true'))
    monkeypatch.setattr(shutdown.subprocess, 'run', gazebocall)
    shutdown.orderly_shutdown({'ROS_DOMAIN_ID': '17', 'GZ_PARTITION': partition})
    if available:
        assert calls == [('/dynamic_obstacles/stop', None),
                         ('/navigate_to_pose/_action/cancel_goal', None),
                         ('/lifecycle_manager_navigation/manage_nodes', ManageLifecycleNodes.Request.SHUTDOWN),
                         ('/lifecycle_manager_localization/manage_nodes', ManageLifecycleNodes.Request.SHUTDOWN)]
    else:
        assert not calls
    assert gazebocall.called == partition.startswith('differential_drive_')
    for call in executor.spin_until_future_complete.call_args_list:
        assert 0 < call.kwargs['timeout_sec'] <= 8.0
    node.create_publisher.return_value.publish.assert_called()
    node.destroy_node.assert_called_once()
    context.try_shutdown.assert_called_once()
