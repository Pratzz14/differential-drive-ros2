"""Regression checks for navigation handoffs and asynchronous action races."""

from types import SimpleNamespace
from unittest.mock import MagicMock

from action_msgs.msg import GoalStatus

from differential_drive_robot_exploration.exploration_runner import ExplorationMission


def test_next_frontier_is_planned_while_goal_is_active():
    node = MagicMock()
    node._remaining_distance = 1.8
    node._ready_frontier = None
    node._selection_waiting = False
    node._active_started = 0.0
    node._last_selection_time = 0.0
    node._sim_seconds.return_value = 3.0
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value={
        'prefetch_distance': 2.0, 'goal_min_separation': 0.7,
        'goal_update_interval': 2.0,
    }[name])
    node._map_pose.return_value = SimpleNamespace(x=0.0, y=0.0)
    node._active_frontier = SimpleNamespace(goal_x=1.8, goal_y=0.0)
    next_frontier = SimpleNamespace(goal_x=3.0, goal_y=0.0)
    node._frontiers.return_value = [node._active_frontier, next_frontier]
    ExplorationMission._prepare_handoff(node)
    node._begin_selection.assert_called_once_with([next_frontier])
    node._active_goal.cancel_goal_async.assert_not_called()


def test_handoff_happens_before_arrival():
    node = MagicMock()
    node._remaining_distance = 0.9
    node._ready_frontier = object()
    node.get_parameter.side_effect = lambda name: SimpleNamespace(value={
        'prefetch_distance': 2.0, 'handoff_distance': 1.0,
    }[name])
    ExplorationMission._prepare_handoff(node)
    node._dispatch_ready_frontier.assert_called_once()
    node._active_goal.cancel_goal_async.assert_not_called()


def test_no_early_handoff_on_long_remaining_path():
    node = MagicMock()
    node._remaining_distance = 4.0
    node.get_parameter.return_value = SimpleNamespace(value=2.0)
    ExplorationMission._prepare_handoff(node)
    node._dispatch_ready_frontier.assert_not_called()
    node._begin_selection.assert_not_called()


def test_old_result_cannot_clear_replacement_goal():
    node = MagicMock()
    active = node._active_goal
    ExplorationMission._navigation_result_done(node, MagicMock(), object(), 1, object())
    assert node._active_goal is active


def test_rejected_replacement_preserves_active_navigation():
    node = MagicMock()
    node._goal_generation = 2
    node._finishing = False
    node._blacklist = []
    node._goals_failed = 0
    active = node._active_goal
    future = MagicMock()
    future.result.return_value = SimpleNamespace(accepted=False)
    ExplorationMission._navigation_goal_done(
        node, future, SimpleNamespace(goal_x=2.0, goal_y=0.0), 2)
    assert node._active_goal is active
    assert node._navigation_pending is False


def test_pending_goal_accepted_after_finish_is_cancelled():
    node = MagicMock()
    node._finishing = True
    future = MagicMock()
    handle = future.result.return_value
    handle.accepted = True
    ExplorationMission._navigation_goal_done(node, future, object(), 2)
    handle.cancel_goal_async.assert_called_once()


def test_cancelled_preempted_goal_is_not_blacklisted():
    node = MagicMock()
    node._finishing = False
    node._navigation_pending = True
    node._goals_preempted = 0
    node._blacklist = []
    future = MagicMock()
    future.result.return_value.status = GoalStatus.STATUS_CANCELED
    ExplorationMission._navigation_result_done(
        node, future, object(), 1, node._active_goal)
    assert node._goals_preempted == 1
    assert node._blacklist == []


def test_success_at_old_endpoint_retries_new_destination():
    node = MagicMock()
    node._finishing = False
    node._navigation_pending = False
    node._goals_succeeded = 0
    node._map_pose.return_value = SimpleNamespace(x=0.0, y=0.0)
    target = SimpleNamespace(goal_x=3.0, goal_y=0.0)
    future = MagicMock()
    future.result.return_value.status = GoalStatus.STATUS_SUCCEEDED
    ExplorationMission._navigation_result_done(node, future, target, 2, node._active_goal)
    assert node._ready_frontier is target
    assert node._goals_succeeded == 0


def test_reached_frontier_is_temporarily_suppressed():
    node = MagicMock()
    node._finishing = False
    node._navigation_pending = False
    node._goals_succeeded = 0
    node._visited = []
    node._sim_seconds.return_value = 5.0
    node._map_pose.return_value = SimpleNamespace(x=3.0, y=0.0)
    target = SimpleNamespace(goal_x=3.0, goal_y=0.0)
    future = MagicMock()
    future.result.return_value.status = GoalStatus.STATUS_SUCCEEDED
    ExplorationMission._navigation_result_done(node, future, target, 2, node._active_goal)
    assert node._visited == [(3.0, 0.0, 15.0)]
    assert node._goals_succeeded == 1


def test_rejected_replacement_does_not_disable_old_feedback():
    node = MagicMock()
    node._active_generation = 1
    node._goal_generation = 2
    feedback = SimpleNamespace(feedback=SimpleNamespace(distance_remaining=1.2))
    ExplorationMission._navigation_feedback(node, feedback, 1)
    assert node._remaining_distance == 1.2
