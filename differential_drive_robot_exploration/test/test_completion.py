"""Completion must not confuse failed recovery or stale data with success."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from action_msgs.msg import GoalStatus
import pytest

from differential_drive_robot_exploration.exploration_runner import ExplorationMission
from test_run_output import grid, result


def mission(**parameters):
    node = MagicMock()
    values = dict(target_coverage=98.0, max_recovery_spins=3, stop_on_incomplete=True)
    values.update(parameters)
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value=values[name])
    node._metrics = {'coverage_percent': 100.0}
    node._recovery_best_coverage = 0.0
    node._recovery_passes = 0
    node._blacklist = []
    node._visited = []
    node._spin_locations = []
    return node


def test_complete_coverage_needs_no_spin():
    node = mission()
    ExplorationMission._after_frontier_verification(node)
    node._finish.assert_called_once_with('coverage_and_frontiers_complete')
    node._start_recovery_spin.assert_not_called()


def test_exhaustion_checks_previously_excluded_frontiers():
    node = mission()
    candidates = [object() for _ in range(6)]
    node._frontiers.return_value = candidates
    ExplorationMission._handle_frontier_exhaustion(node)
    node._frontiers.assert_called_once_with(apply_blacklist=False)
    node._begin_selection.assert_called_once_with(candidates, verify_exhaustion=True)
    node._finish.assert_not_called()


def test_below_target_stops_as_incomplete_after_budget():
    node = mission()
    node._metrics = {'coverage_percent': 90.0}
    node._recovery_best_coverage = 90.0
    node._recovery_passes = 3
    ExplorationMission._after_frontier_verification(node)
    node._finish.assert_called_once_with('frontiers_exhausted_below_target', status='incomplete')


def test_spin_is_not_repeated_at_same_location():
    node = mission()
    node._map_pose.return_value = SimpleNamespace(x=1.0, y=2.0)
    node._spin_locations = [(1.1, 2.0)]
    ExplorationMission._start_recovery_spin(node)
    node._spin.send_goal_async.assert_not_called()


@pytest.mark.parametrize('status,successes,failures', [
    (GoalStatus.STATUS_SUCCEEDED, 1, 0),
    (GoalStatus.STATUS_ABORTED, 0, 1),
])
def test_spin_outcomes_are_distinct(status, successes, failures):
    node = mission()
    node._recovery_spins_succeeded = node._recovery_spins_failed = 0
    future = Future()
    future.set_result(SimpleNamespace(status=status))
    ExplorationMission._spin_result_done(node, future, node._active_spin)
    assert node._recovery_spins_succeeded == successes
    assert node._recovery_spins_failed == failures


def test_late_spin_acceptance_after_finish_is_cancelled():
    node = mission()
    node._finishing = True
    node._cancel_futures = []
    handle = MagicMock(accepted=True)
    future = Future()
    future.set_result(handle)
    ExplorationMission._spin_goal_done(node, future, 1)
    handle.cancel_goal_async.assert_called_once()
    assert len(node._cancel_futures) == 1


def stopping_node():
    node = mission()
    node._finishing = True
    node._finish_state = 'stopping'
    node._navigation_pending = node._spin_pending = False
    node._cancel_futures = []
    node._stationary_since = 99.0
    node._last_odom_wall = 100.0
    node._last_map_wall = 100.0
    node._finish_started_wall = 98.0
    node._finish_reason = 'coverage_and_frontiers_complete'
    node._finish_status = 'succeeded'
    node._warnings = []
    return node


def test_finish_waits_for_cancellation_ack():
    node = stopping_node()
    node._cancel_futures = [Future()]
    with patch('differential_drive_robot_exploration.exploration_runner.time.monotonic', return_value=100.0):
        ExplorationMission._wall_tick(node)
    assert node._finish_state == 'stopping'
    node._analysis_pool.submit.assert_not_called()


def test_stop_timeout_saves_but_never_claims_success():
    node = stopping_node()
    node._stationary_since = None
    node._finish_started_wall = 90.0
    with patch('differential_drive_robot_exploration.exploration_runner.time.monotonic', return_value=100.0):
        ExplorationMission._wall_tick(node)
    assert node._finish_status == 'incomplete'
    assert node._finish_reason == 'stop_confirmation_timeout'
    assert node._finish_state == 'saving'


def test_stale_map_has_bounded_exit():
    node = mission()
    node._finishing = False
    node._last_map_wall = 0.0
    with patch('differential_drive_robot_exploration.exploration_runner.time.monotonic', return_value=100.0):
        ExplorationMission._wall_tick(node)
    node._finish.assert_called_once_with('map_updates_unavailable', status='incomplete')


def test_hold_keeps_node_alive_without_new_goals():
    node = mission()
    node._finishing = True
    node._finish_state = 'finished'
    ExplorationMission._wall_tick(node)
    node._stop_pub.publish.assert_called_once()
    node._shutdown_once.assert_not_called()
    node._send_navigation_goal.assert_not_called()


def test_map_save_failure_is_reported_not_thrown(tmp_path):
    node = mission(record_results=True)
    node._alignment = None
    metadata = result(tmp_path)
    metadata['target_coverage_percent'] = 98.0
    node._result_metadata.return_value = metadata
    with patch('differential_drive_robot_exploration.exploration_runner.write_map', side_effect=OSError('full disk')):
        saved = ExplorationMission._save_snapshot(node, grid(), 'complete', 'succeeded')
    assert saved['status'] == 'save_failed'
    assert saved['map_status'] == 'save_failed'
    assert 'full disk' in saved['warnings'][0]
    assert (tmp_path / 'result.json').is_file()


def test_scoring_failure_still_saves_the_map(tmp_path):
    node = mission(record_results=True, start_x=-5.0, start_y=0.0)
    metadata = result(tmp_path)
    metadata['target_coverage_percent'] = 98.0
    node._result_metadata.return_value = metadata
    with patch('differential_drive_robot_exploration.exploration_runner.score_grid', side_effect=ValueError('bad truth')):
        saved = ExplorationMission._save_snapshot(node, grid(), 'complete', 'succeeded')
    assert saved['status'] == 'incomplete'
    assert saved['completion_reason'] == 'metrics_unavailable'
    assert (tmp_path / 'map.yaml').is_file()


def test_report_write_failure_is_visible(tmp_path):
    node = mission(record_results=True)
    node._alignment = None
    metadata = result(tmp_path)
    metadata['target_coverage_percent'] = 98.0
    node._result_metadata.return_value = metadata
    with patch('differential_drive_robot_exploration.exploration_runner.write_report', side_effect=OSError('read only')):
        saved = ExplorationMission._save_snapshot(node, grid(), 'complete', 'succeeded')
    assert saved['status'] == 'save_failed'
    assert saved['map_status'] == 'saved_direct'
    assert 'Report save failed' in saved['warnings'][0]


def test_pending_navigation_acceptance_has_deadline():
    node = mission()
    node._finishing = False
    node._initialized = True
    node._last_map_wall = 100.0
    node._navigation_pending = True
    node._navigation_request_wall = 80.0
    with patch('differential_drive_robot_exploration.exploration_runner.time.monotonic', return_value=100.0):
        ExplorationMission._wall_tick(node)
    node._finish.assert_called_once_with('navigation_acceptance_timeout', status='incomplete')


@pytest.mark.parametrize('finishing', [False, True])
def test_finalization_gates_late_nonzero_velocity(finishing):
    from geometry_msgs.msg import Twist
    node = mission()
    node._finishing = finishing
    command = Twist()
    command.linear.x = 0.5
    command.angular.z = 0.4
    ExplorationMission._velocity_callback(node, command)
    published = node._stop_pub.publish.call_args.args[0]
    assert published.linear.x == (0.0 if finishing else 0.5)
    assert published.angular.z == (0.0 if finishing else 0.4)


@pytest.mark.parametrize('requested,expected', [(False, 0), (True, 1)])
def test_shutdown_race_is_not_logged_as_a_mission_exception(requested, expected):
    from differential_drive_robot_exploration import exploration_runner
    node = MagicMock()
    node._shutdown_requested = requested
    node.exit_code = 1
    with patch.object(exploration_runner, 'ExplorationMission', return_value=node), \
            patch.object(exploration_runner.rclpy, 'init'), \
            patch.object(exploration_runner.rclpy, 'ok', return_value=False), \
            patch.object(exploration_runner.rclpy, 'spin', side_effect=RuntimeError('context invalid')):
        with pytest.raises(SystemExit) as error:
            exploration_runner.main()
    assert error.value.code == expected
    node.get_logger.assert_not_called()
