"""Send the fixed Nav2 goal and record a reproducible trial result."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'unknown',
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


class MissionRunner(Node):
    def __init__(self) -> None:
        super().__init__('navigation_mission')
        self.declare_parameter('seed', 0)
        self.declare_parameter('layout_file', '')
        self.declare_parameter(
            'results_directory', '/tmp/differential_drive_navigation/results'
        )
        self.declare_parameter('record_results', True)
        self.declare_parameter('mission_timeout', 180.0)
        self.declare_parameter('auto_goal', False)
        self.declare_parameter('start_x', -5.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('localization_position_variance_limit', 0.25)
        self.declare_parameter('localization_yaw_variance_limit', 0.20)

        initial_pose_qos = QoSProfile(depth=1)
        initial_pose_qos.reliability = ReliabilityPolicy.RELIABLE
        initial_pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', initial_pose_qos
        )
        self.navigation_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.amcl_state_client = self.create_client(GetState, '/amcl/get_state')
        self.navigation_lifecycle_client = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes'
        )
        self.create_subscription(Odometry, '/odometry/filtered', self._odometry_callback, 20)
        self.create_subscription(PathMessage, '/plan', self._plan_callback, 10)
        self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback, 10
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )

        self._last_odometry: tuple[float, float] | None = None
        self._distance_traveled = 0.0
        self._planned_path_length = 0.0
        self._amcl_pose: tuple[float, float] | None = None
        self._amcl_samples: deque[tuple[float, float, float, float, float]] = deque(
            maxlen=10
        )
        self._initial_pose_started = False
        self._scan_received = False
        self._first_scan_after_initial: float | None = None

    def _odometry_callback(self, message: Odometry) -> None:
        current = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self._last_odometry is not None:
            step = math.dist(self._last_odometry, current)
            if step < 1.0:
                self._distance_traveled += step
        self._last_odometry = current

    def _plan_callback(self, message: PathMessage) -> None:
        self._planned_path_length = sum(
            math.dist(
                (first.pose.position.x, first.pose.position.y),
                (second.pose.position.x, second.pose.position.y),
            )
            for first, second in zip(message.poses, message.poses[1:])
        )

    def _scan_callback(self, _message: LaserScan) -> None:
        self._scan_received = True
        if self._initial_pose_started and self._first_scan_after_initial is None:
            self._first_scan_after_initial = time.monotonic()

    def _amcl_callback(self, message: PoseWithCovarianceStamped) -> None:
        self._amcl_pose = (message.pose.pose.position.x, message.pose.pose.position.y)
        covariance = message.pose.covariance
        self._amcl_samples.append(
            (
                time.monotonic(),
                covariance[0],
                covariance[7],
                covariance[35],
                math.hypot(
                    message.pose.pose.position.x
                    - float(self.get_parameter('start_x').value),
                    message.pose.pose.position.y
                    - float(self.get_parameter('start_y').value),
                ),
            )
        )

    def localization_ready(self) -> bool:
        if self._first_scan_after_initial is None or not self._amcl_samples:
            return False
        sample = self._amcl_samples[-1]
        received_at, variance_x, variance_y, variance_yaw, start_error = sample
        if received_at < self._first_scan_after_initial:
            return False
        position_limit = float(
            self.get_parameter('localization_position_variance_limit').value
        )
        yaw_limit = float(
            self.get_parameter('localization_yaw_variance_limit').value
        )
        if max(variance_x, variance_y) > position_limit or variance_yaw > yaw_limit:
            return False
        if start_error > 0.50:
            return False
        return self._tf_buffer.can_transform(
            'map',
            'base_link',
            Time(),
            timeout=Duration(seconds=0.1),
        )

    def _pose_message(self, x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # These are variances, not standard deviations. Keep the simulated
        # initial particle cloud tight around the exact Gazebo spawn pose.
        message.pose.covariance[0] = 0.0025
        message.pose.covariance[7] = 0.0025
        message.pose.covariance[35] = 0.0012
        return message

    def publish_initial_pose(self) -> None:
        self._amcl_pose = None
        self._amcl_samples.clear()
        self._first_scan_after_initial = None
        self._initial_pose_started = True
        pose = self._pose_message(
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
            float(self.get_parameter('start_yaw').value),
        )
        self.get_logger().info('Publishing the fixed initial pose')
        self.initial_pose_publisher.publish(pose)

    def reset_mission_metrics(self) -> None:
        self._last_odometry = None
        self._distance_traveled = 0.0
        self._planned_path_length = 0.0

    def goal_message(self) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.pose.pose.position.y = float(self.get_parameter('goal_y').value)
        yaw = float(self.get_parameter('goal_yaw').value)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.behavior_tree = ''
        return goal

    def write_result(self, result: dict) -> Path | None:
        if not bool(self.get_parameter('record_results').value):
            return None
        result.setdefault('finished_at', datetime.now(timezone.utc).isoformat())
        layout_file = Path(str(self.get_parameter('layout_file').value))
        layout = {}
        if layout_file.is_file():
            layout = json.loads(layout_file.read_text(encoding='utf-8'))
        result['layout'] = layout
        output_directory = Path(str(self.get_parameter('results_directory').value))
        output_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        output_path = output_directory / f"trial_{timestamp}_seed_{result['seed']}.json"
        output_path.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        return output_path


def _wait_until(node: Node, predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if predicate():
            return True
    return False


def _cancel_goal(node: MissionRunner, goal_handle, result_future) -> str | None:
    """Cancel an active goal and wait until Nav2 reaches a terminal state."""
    cancel_future = goal_handle.cancel_goal_async()
    if not _wait_until(node, cancel_future.done, 5.0):
        return 'cancel request did not receive a response'
    cancel_response = cancel_future.result()
    if cancel_response is None or not cancel_response.goals_canceling:
        if result_future.done():
            return None
        return 'Nav2 rejected the cancel request'
    if not result_future.done() and not _wait_until(node, result_future.done, 5.0):
        return 'goal did not reach a terminal state after cancellation'
    return None


def run_mission(node: MissionRunner) -> bool:
    seed = int(node.get_parameter('seed').value)
    result = {
        'seed': seed,
        'started_at': datetime.now(timezone.utc).isoformat(),
        'status': 'startup_failed',
        'failure_reason': '',
        'elapsed_seconds': 0.0,
        'distance_traveled_m': 0.0,
        'latest_planned_path_m': 0.0,
        'final_pose_error_m': None,
    }

    node.get_logger().info(f'Starting navigation trial with seed {seed}')
    node.get_logger().info('Waiting for simulation clock, scan, odometry and active AMCL')
    state_future = None

    def startup_ready():
        nonlocal state_future
        if state_future is None and node.amcl_state_client.service_is_ready():
            state_future = node.amcl_state_client.call_async(GetState.Request())
        if state_future is None or not state_future.done():
            return False
        if state_future.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
            state_future = None
            return False
        return (
            node.get_clock().now().nanoseconds > 0
            and node._scan_received
            and node._last_odometry is not None
            and node.initial_pose_publisher.get_subscription_count() > 0
            and node._tf_buffer.can_transform('odom', 'base_link', Time())
        )

    if not _wait_until(
        node,
        startup_ready,
        60.0,
    ):
        result['failure_reason'] = 'Simulation sensors, odometry or active AMCL unavailable'
        node.write_result(result)
        node.get_logger().error(result['failure_reason'])
        return False

    node.publish_initial_pose()
    if not _wait_until(node, node.localization_ready, 30.0):
        result['failure_reason'] = (
            'localization did not become scan-confirmed, bounded, and TF-ready'
        )
        node.write_result(result)
        node.get_logger().error(result['failure_reason'])
        return False

    node.get_logger().info('Localization ready; activating Nav2')
    if not _wait_until(node, node.navigation_lifecycle_client.service_is_ready, 30.0):
        result['failure_reason'] = 'Navigation lifecycle manager unavailable'
        node.write_result(result)
        node.get_logger().error(result['failure_reason'])
        return False
    request = ManageLifecycleNodes.Request()
    request.command = ManageLifecycleNodes.Request.STARTUP
    activation = node.navigation_lifecycle_client.call_async(request)
    if not _wait_until(node, activation.done, 90.0) or not activation.result().success:
        result['failure_reason'] = 'Nav2 lifecycle activation failed'
        node.write_result(result)
        node.get_logger().error(result['failure_reason'])
        return False

    if not _wait_until(node, node.navigation_client.server_is_ready, 120.0):
        result['failure_reason'] = 'navigate_to_pose action server unavailable'
        node.write_result(result)
        node.get_logger().error(result['failure_reason'])
        return False

    if not bool(node.get_parameter('auto_goal').value):
        node.get_logger().info(
            'Localization initialized at the fixed start pose; '
            'waiting for a manual RViz 2D Goal Pose'
        )
        return True

    goal = node.goal_message()
    node.reset_mission_metrics()
    send_future = node.navigation_client.send_goal_async(goal)
    if not _wait_until(node, send_future.done, 15.0):
        result['failure_reason'] = 'timed out while sending navigation goal'
        node.write_result(result)
        return False
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        result['failure_reason'] = 'navigation goal was rejected'
        node.write_result(result)
        node.get_logger().error(result['failure_reason'])
        return False

    started = time.monotonic()
    result_future = goal_handle.get_result_async()
    timeout = float(node.get_parameter('mission_timeout').value)
    if not _wait_until(node, result_future.done, timeout):
        result['status'] = 'timed_out'
        result['failure_reason'] = f'navigation exceeded {timeout:.1f} seconds'
        cancellation_error = _cancel_goal(node, goal_handle, result_future)
        if cancellation_error is not None:
            result['failure_reason'] += f'; {cancellation_error}'
    else:
        response = result_future.result()
        result['status'] = STATUS_NAMES.get(response.status, f'status_{response.status}')
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            error_code = getattr(response.result, 'error_code', 0)
            error_msg = getattr(response.result, 'error_msg', '')
            result['failure_reason'] = error_msg or f'Nav2 error code {error_code}'

    result['elapsed_seconds'] = round(time.monotonic() - started, 3)
    result['distance_traveled_m'] = round(node._distance_traveled, 3)
    result['latest_planned_path_m'] = round(node._planned_path_length, 3)
    goal_xy = (
        float(node.get_parameter('goal_x').value),
        float(node.get_parameter('goal_y').value),
    )
    if node._amcl_pose is not None:
        result['final_pose_error_m'] = round(math.dist(node._amcl_pose, goal_xy), 3)
    result['finished_at'] = datetime.now(timezone.utc).isoformat()
    output_path = node.write_result(result)
    message = f"Trial finished with status {result['status']}"
    if output_path is not None:
        message += f'; result: {output_path}'
    node.get_logger().info(message)
    return result['status'] == 'succeeded'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionRunner()
    try:
        succeeded = run_mission(node)
    except KeyboardInterrupt:
        return
    except Exception as exception:  # Ensure unexpected failures remain reproducible.
        failure_reason = f'{type(exception).__name__}: {exception}'
        node.get_logger().error(f'Mission runner failed: {failure_reason}')
        failure_result = {
            'seed': int(node.get_parameter('seed').value),
            'started_at': datetime.now(timezone.utc).isoformat(),
            'status': 'internal_error',
            'failure_reason': failure_reason,
            'elapsed_seconds': 0.0,
            'distance_traveled_m': round(node._distance_traveled, 3),
            'latest_planned_path_m': round(node._planned_path_length, 3),
            'final_pose_error_m': None,
        }
        try:
            node.write_result(failure_result)
        except Exception as write_exception:
            node.get_logger().error(
                f'Unable to record internal error result: {write_exception}'
            )
        succeeded = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if succeeded else 1)


if __name__ == '__main__':
    main()
