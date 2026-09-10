"""Frontier exploration mission coordinator and metric recorder."""
# flake8: noqa

from __future__ import annotations

from datetime import datetime, timezone
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .frontier import GridSpec, extract_frontiers, path_length
from .run_output import summary_text, write_map, write_report
from .truth import (
    Alignment,
    freeze_alignment,
    parse_static_boxes,
    Pose2D,
    pose_from_pose_array,
    score_grid,
)


def _yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quaternion_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ExplorationMission(Node):
    def __init__(self) -> None:
        super().__init__('exploration_mission')
        self.declare_parameter('seed', 0)
        self.declare_parameter('layout_file', '')
        self.declare_parameter('world_file', '')
        self.declare_parameter('output_directory', str(Path.home() / 'ROS_Maps' / 'exploration'))
        self.declare_parameter('record_results', True)
        self.declare_parameter('target_coverage', 98.0)
        self.declare_parameter('mission_timeout', 0.0)
        self.declare_parameter('goal_timeout', 45.0)
        self.declare_parameter('frontier_min_cells', 5)
        self.declare_parameter('exhaustion_cycles', 3)
        self.declare_parameter('max_recovery_spins', 3)
        self.declare_parameter('stop_on_incomplete', True)
        self.declare_parameter('finish_behavior', 'hold')
        if self.get_parameter('finish_behavior').value not in ('hold', 'shutdown'):
            raise ValueError('finish_behavior must be hold or shutdown')
        self.declare_parameter('start_x', -5.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('truth_pose_index', 6)
        self.declare_parameter('prefetch_distance', 2.0)
        self.declare_parameter('handoff_distance', 1.0)
        self.declare_parameter('goal_min_separation', 0.7)
        self.declare_parameter('goal_update_interval', 2.0)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._map: OccupancyGrid | None = None
        self._map_revision = 0
        self._last_map_wall = time.monotonic()
        self._analysis_revision = -1
        self._analysis_future = None
        self._analysis_pool = ThreadPoolExecutor(max_workers=1)
        self._cached_frontiers = []
        self._truth_pose: Pose2D | None = None
        self._alignment: Alignment | None = None
        self._boxes = parse_static_boxes(str(self.get_parameter('world_file').value))
        self._last_truth: Pose2D | None = None
        self._distance = 0.0
        self._last_distance_time: float | None = None
        self._metrics: dict[str, float | int | None] = {}
        self._started_sim_ns: int | None = None
        self._finished_sim_ns: int | None = None
        self._started_wall = time.monotonic()
        self._started_at = datetime.now(timezone.utc).isoformat()

        self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self.create_subscription(PoseArray, '/ground_truth/poses', self._truth_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_callback, 10)
        self._frontier_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 10)
        self._status_pub = self.create_publisher(String, '/exploration/status', map_qos)
        self._summary_pub = self.create_publisher(Marker, '/exploration/summary', map_qos)
        self._final_map_pub = self.create_publisher(OccupancyGrid, '/exploration/final_map', map_qos)
        self._stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_exploration', self._velocity_callback, 10)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._navigate = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._compute_path = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self._spin = ActionClient(self, Spin, '/spin')

        self._active_goal = None
        self._active_frontier = None
        self._active_started = 0.0
        self._goal_generation = 0
        self._active_generation = 0
        self._navigation_pending = False
        self._navigation_request_wall = 0.0
        self._cancel_requested = False
        self._remaining_distance = None
        self._ready_frontier = None
        self._last_selection_time = -math.inf
        self._goals_preempted = 0
        self._active_spin = None
        self._spin_pending = False
        self._blacklist: list[tuple[float, float]] = []
        self._visited = []
        self._selection_queue = []
        self._selection_results: list[tuple[object, float]] = []
        self._selection_waiting = False
        self._verifying_exhaustion = False
        self._selection_started_wall = 0.0
        self._empty_cycles = 0
        self._last_empty_revision = -1
        self._recovery_spins = 0
        self._recovery_spins_succeeded = 0
        self._recovery_spins_failed = 0
        self._recovery_passes = 0
        self._recovery_best_coverage = 0.0
        self._spin_locations = []
        self._spin_started_wall = 0.0
        self._spin_generation = 0
        self._goals_attempted = 0
        self._goals_succeeded = 0
        self._goals_failed = 0
        self._initialized = False
        self._finishing = False
        self._shutdown_requested = False
        self._finish_state = 'exploring'
        self._cancel_futures = []
        self._stationary_since = None
        self._last_odom_wall = 0.0
        self._final_future = None
        self._final_snapshot = None
        self._warnings = []
        self.exit_code = 1
        self._timer = self.create_timer(0.2, self._control_tick)
        # Finish deadlines still run when Gazebo /clock stops or is paused.
        self._wall_timer = self.create_timer(
            0.2, self._wall_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _velocity_callback(self, message):
        # Gate the smoother's output, rather than racing it with periodic zeros
        # if cancellation is delayed or another RViz goal arrives during hold.
        self._stop_pub.publish(Twist() if self._finishing else message)

    def _odom_callback(self, message):
        now = time.monotonic()
        velocity = message.twist.twist
        stopped = math.hypot(velocity.linear.x, velocity.linear.y) < 0.02 and abs(velocity.angular.z) < 0.03
        if stopped:
            if self._stationary_since is None or now - self._last_odom_wall > 2.0:
                self._stationary_since = now
        else:
            self._stationary_since = None
        self._last_odom_wall = now

    def _truth_callback(self, message: PoseArray) -> None:
        pose = pose_from_pose_array(message, int(self.get_parameter('truth_pose_index').value))
        if pose is None:
            return
        self._truth_pose = pose
        if self._last_truth is not None and self._initialized and self._finish_state in ('exploring', 'stopping'):
            step = math.dist((pose.x, pose.y), (self._last_truth.x, self._last_truth.y))
            if step < 1.0:
                self._distance += step
        self._last_truth = pose
        if self._initialized and self._started_sim_ns is not None:
            self._last_distance_time = self.get_clock().now().nanoseconds / 1e9

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._map = message
        self._map_revision += 1
        self._last_map_wall = time.monotonic()
        self._try_initialize()

    def _map_pose(self) -> Pose2D | None:
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        return Pose2D(
            transform.transform.translation.x,
            transform.transform.translation.y,
            _quaternion_yaw(transform.transform.rotation),
        )

    def _try_initialize(self) -> None:
        if self._initialized or self._map is None or self._truth_pose is None:
            return
        map_pose = self._map_pose()
        if map_pose is None:
            return
        self._alignment = freeze_alignment(map_pose, self._truth_pose)
        self._started_sim_ns = self.get_clock().now().nanoseconds
        self._last_truth = self._truth_pose
        self._last_distance_time = self._started_sim_ns / 1e9
        self._initialized = True
        self.get_logger().info('SLAM map, Gazebo truth, and frozen map alignment are ready')
        self._publish_status('exploring')

    def _update_metrics(self) -> None:
        # Only the worker touches the immutable snapshot. ROS callbacks remain
        # responsive while full-arena scoring and frontier extraction run.
        if self._analysis_future is not None:
            if not self._analysis_future.done():
                return
            revision, self._metrics, self._cached_frontiers = self._analysis_future.result()
            self._analysis_revision = revision
            self._analysis_future = None
            self._publish_frontiers(self._cached_frontiers)
            self._publish_status('exploring')
        if self._map is None or self._alignment is None:
            return
        if self._analysis_revision == self._map_revision:
            return
        info = self._map.info
        spec = GridSpec(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            _quaternion_yaw(info.origin.orientation),
        )
        self._analysis_future = self._analysis_pool.submit(
            self._analyze_map, self._map_revision, list(self._map.data), spec,
            self._boxes, self._alignment,
            (float(self.get_parameter('start_x').value), float(self.get_parameter('start_y').value)),
            int(self.get_parameter('frontier_min_cells').value),
        )

    @staticmethod
    def _analyze_map(revision, data, spec, boxes, alignment, start, min_cells):
        return (
            revision, score_grid(data, spec, boxes, alignment, start),
            extract_frontiers(data, spec, min_cells=min_cells),
        )

    def _publish_status(self, status: str) -> None:
        message = String()
        coverage = self._metrics.get('coverage_percent')
        message.data = status if coverage is None else f'{status}: coverage={coverage:.2f}%'
        self._status_pub.publish(message)

    def _frontiers(self, *, apply_blacklist: bool = True):
        candidates = self._cached_frontiers
        if not apply_blacklist:
            return candidates
        now = self._sim_seconds()
        self._visited = [(x, y, until) for x, y, until in self._visited if until > now]
        return [frontier for frontier in candidates if all(
            math.dist((frontier.goal_x, frontier.goal_y), blacklisted) > 0.5
            for blacklisted in self._blacklist
        ) and all(math.dist((frontier.goal_x, frontier.goal_y), (x, y)) > 0.4
                  for x, y, _ in self._visited)]

    def _control_tick(self) -> None:
        if self._finishing:
            return
        self._try_initialize()
        if not self._initialized or not self._navigate.server_is_ready():
            return
        self._update_metrics()
        if self._started_sim_ns is not None:
            elapsed = (self.get_clock().now().nanoseconds - self._started_sim_ns) / 1e9
            timeout = float(self.get_parameter('mission_timeout').value)
            if timeout > 0.0 and elapsed >= timeout:
                self._finish('timeout', status='timed_out')
                return
        if self._spin_pending or self._active_spin is not None:
            return
        if self._navigation_pending:
            return
        if self._active_goal is not None:
            if self._cancel_requested:
                return
            if self._sim_seconds() - self._active_started > float(self.get_parameter('goal_timeout').value):
                self._cancel_requested = True
                self._active_goal.cancel_goal_async()
                return
            self._prepare_handoff()
            return
        if self._selection_waiting:
            return
        if self._ready_frontier is not None:
            self._dispatch_ready_frontier()
            return
        if self._analysis_revision < 0:
            return
        candidates = self._frontiers()
        if not candidates:
            # Count exhaustion only once for each new SLAM map. This prevents a
            # fast timer from declaring completion while the map is unchanged.
            if self._analysis_revision <= self._last_empty_revision:
                return
            self._last_empty_revision = self._analysis_revision
            self._empty_cycles += 1
            if self._empty_cycles >= int(self.get_parameter('exhaustion_cycles').value):
                self._handle_frontier_exhaustion()
            return
        self._empty_cycles = 0
        self._begin_selection(candidates)

    def _sim_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _prepare_handoff(self):
        remaining = self._remaining_distance
        if remaining is None or remaining > float(self.get_parameter('prefetch_distance').value):
            return
        if self._ready_frontier is not None:
            if remaining <= float(self.get_parameter('handoff_distance').value):
                self._dispatch_ready_frontier()
            return
        if self._selection_waiting:
            return
        interval = float(self.get_parameter('goal_update_interval').value)
        if self._sim_seconds() - max(self._active_started, self._last_selection_time) < interval:
            return
        separation = float(self.get_parameter('goal_min_separation').value)
        pose = self._map_pose()
        if pose is None:
            return
        candidates = [f for f in self._frontiers() if
                      math.dist((f.goal_x, f.goal_y),
                                (self._active_frontier.goal_x, self._active_frontier.goal_y)) >= separation
                      and math.dist((f.goal_x, f.goal_y), (pose.x, pose.y)) >= separation]
        if candidates:
            self._begin_selection(candidates)

    def _dispatch_ready_frontier(self):
        frontier = self._ready_frontier
        self._ready_frontier = None
        # A disappearing frontier has already been observed while travelling.
        # Re-evaluate instead of navigating to an obsolete cached destination.
        if any(math.dist((f.goal_x, f.goal_y), (frontier.goal_x, frontier.goal_y)) < 0.4
               for f in self._frontiers()):
            self._send_navigation_goal(frontier)

    def _candidate_score(self, frontier, distance):
        pose = self._map_pose()
        heading_weight = 1.0
        if pose is not None:
            heading = math.atan2(frontier.goal_y - pose.y, frontier.goal_x - pose.x)
            heading_weight = 0.65 + 0.35 * math.cos(heading - pose.yaw)
        # Saturate the near-goal bonus: tiny nearby boundaries should not
        # dominate broad frontiers that extend the map in our travel direction.
        return math.sqrt(frontier.information_gain_m) * heading_weight / max(1.5, distance)

    def _handle_frontier_exhaustion(self) -> None:
        # Cooldowns and failed-path blacklists are not evidence of completeness.
        # Revalidate every candidate, not just the normal top-four shortlist.
        candidates = self._frontiers(apply_blacklist=False)
        if candidates:
            self._blacklist.clear()
            self._visited.clear()
            self._begin_selection(candidates, verify_exhaustion=True)
            return
        self._after_frontier_verification()

    def _after_frontier_verification(self):
        coverage = self._metrics.get('coverage_percent')
        target = float(self.get_parameter('target_coverage').value)
        if coverage is not None and coverage >= target:
            self._finish('coverage_and_frontiers_complete')
            return
        if coverage is not None and coverage > self._recovery_best_coverage + 0.1:
            self._recovery_passes = 0
            self._recovery_best_coverage = coverage
        if self._recovery_passes < int(self.get_parameter('max_recovery_spins').value):
            self._recovery_passes += 1
            self._empty_cycles = 0
            self._blacklist.clear()
            self._visited.clear()
            self._start_recovery_spin()
        elif bool(self.get_parameter('stop_on_incomplete').value):
            self._finish('frontiers_exhausted_below_target', status='incomplete')
        else:
            self.get_logger().warning(
                f'No reachable frontier but coverage is {coverage}% (target {target}%); '
                'continuing recovery because stop_on_incomplete is false'
            )
            self._publish_status('stalled_retrying')
            self._empty_cycles = 0
            self._recovery_passes = 0

    def _start_recovery_spin(self) -> None:
        pose = self._map_pose()
        if pose is None or any(math.dist((pose.x, pose.y), point) < 0.5 for point in self._spin_locations):
            self._publish_status('rechecking_frontiers')
            return
        self._spin_locations.append((pose.x, pose.y))
        self._recovery_spins += 1
        self._empty_cycles = 0
        self._blacklist.clear()
        self._visited.clear()
        self._publish_status(f'recovering_{self._recovery_spins}')
        if not self._spin.server_is_ready():
            self._recovery_spins_failed += 1
            self.get_logger().warning('Spin recovery server is unavailable; retrying frontiers')
            return
        goal = Spin.Goal()
        goal.target_yaw = float(2.0 * math.pi)
        goal.time_allowance.sec = 30
        self._spin_pending = True
        self._spin_generation += 1
        generation = self._spin_generation
        self._spin_started_wall = time.monotonic()
        future = self._spin.send_goal_async(goal)
        future.add_done_callback(lambda result: self._spin_goal_done(result, generation))

    def _spin_goal_done(self, future, generation) -> None:
        handle = future.result()
        if self._finishing or generation != self._spin_generation:
            if handle is not None and handle.accepted:
                self._cancel_futures.append(handle.cancel_goal_async())
            self._spin_pending = False
            return
        self._spin_pending = False
        if handle is None or not handle.accepted:
            self._recovery_spins_failed += 1
            self.get_logger().warning('Recovery spin was rejected')
            return
        self._active_spin = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda result, item=handle: self._spin_result_done(result, item))

    def _spin_result_done(self, future, handle) -> None:
        if self._active_spin is not handle:
            return
        response = future.result()
        if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
            self._recovery_spins_failed += 1
            self.get_logger().warning('Recovery spin did not complete successfully')
        else:
            self._recovery_spins_succeeded += 1
        if self._active_spin is handle:
            self._active_spin = None
        self._last_empty_revision = self._map_revision

    def _pose_stamped(self, frontier) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = frontier.goal_x
        pose.pose.position.y = frontier.goal_y
        pose.pose.orientation.z, pose.pose.orientation.w = _yaw_to_quaternion(frontier.yaw)
        return pose

    def _publish_frontiers(self, frontiers) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for marker_id, frontier in enumerate(frontiers, start=1):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontier.goal_x
            marker.pose.position.y = frontier.goal_y
            marker.pose.position.z = 0.04
            marker.scale.x = marker.scale.y = marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.a = 0.9
            markers.markers.append(marker)
        self._frontier_pub.publish(markers)

    def _begin_selection(self, candidates, *, verify_exhaustion=False) -> None:
        if self._finishing or self._navigation_pending:
            return
        self._last_selection_time = self._sim_seconds()
        self._selection_results = []
        pose = self._map_pose()
        if pose is None or not self._compute_path.server_is_ready():
            return
        self._verifying_exhaustion = verify_exhaustion
        self._selection_started_wall = time.monotonic()
        self._selection_queue = sorted(
            candidates, key=lambda f: -self._candidate_score(
                f, math.dist((pose.x, pose.y), (f.goal_x, f.goal_y))))
        if not verify_exhaustion:
            self._selection_queue = self._selection_queue[:4]
        self._selection_waiting = True
        self._request_path()

    def _request_path(self) -> None:
        if self._finishing:
            self._selection_waiting = False
            return
        if not self._selection_queue:
            self._selection_waiting = False
            if not self._selection_results:
                if self._verifying_exhaustion:
                    self._verifying_exhaustion = False
                    self._after_frontier_verification()
                return
            self._verifying_exhaustion = False
            frontier, distance = max(
                self._selection_results,
                key=lambda item: self._candidate_score(item[0], item[1]),
            )
            self._ready_frontier = frontier
            if self._active_goal is None:
                self._dispatch_ready_frontier()
            return
        frontier = self._selection_queue.pop(0)
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose_stamped(frontier)
        goal.use_start = False
        future = self._compute_path.send_goal_async(goal)
        future.add_done_callback(lambda result, item=frontier: self._path_goal_done(result, item))

    def _path_goal_done(self, future, frontier) -> None:
        handle = future.result()
        if self._finishing:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self._blacklist.append((frontier.goal_x, frontier.goal_y))
            self._request_path()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda result, item=frontier: self._path_result_done(result, item))

    def _path_result_done(self, future, frontier) -> None:
        if self._finishing:
            return
        response = future.result()
        if (response is not None and response.status == GoalStatus.STATUS_SUCCEEDED
                and response.result.path.poses):
            points = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in response.result.path.poses
            ]
            distance = path_length(points)
            self._selection_results.append((frontier, distance))
        else:
            self._blacklist.append((frontier.goal_x, frontier.goal_y))
        self._request_path()

    def _send_navigation_goal(self, frontier) -> None:
        if self._finishing or self._navigation_pending:
            return
        self._goals_attempted += 1
        self._goal_generation += 1
        generation = self._goal_generation
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(frontier)
        self._navigation_pending = True
        self._navigation_request_wall = time.monotonic()
        # NavigateToPose supports replacement goals. Do not cancel the current
        # navigation first: Nav2 updates its running behaviour tree and path.
        future = self._navigate.send_goal_async(
            goal, feedback_callback=lambda feedback, token=generation:
            self._navigation_feedback(feedback, token))
        future.add_done_callback(
            lambda result, item=frontier, token=generation:
            self._navigation_goal_done(result, item, token)
        )

    def _navigation_goal_done(self, future, frontier, generation: int) -> None:
        handle = future.result()
        self._navigation_pending = False
        if self._finishing or generation != self._goal_generation:
            if handle is not None and handle.accepted:
                self._cancel_futures.append(handle.cancel_goal_async())
            return
        if handle is None or not handle.accepted:
            self._goals_failed += 1
            self._blacklist.append((frontier.goal_x, frontier.goal_y))
            return
        if self._active_goal is not None:
            self._goals_preempted += 1
            self.get_logger().info('Rolling frontier handoff: updating goal during travel')
        self._active_goal = handle
        self._active_generation = generation
        self._active_frontier = frontier
        self._active_started = self._sim_seconds()
        self._remaining_distance = None
        self._cancel_requested = False
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result, item=frontier, token=generation, goal_handle=handle:
            self._navigation_result_done(result, item, token, goal_handle)
        )

    def _navigation_result_done(self, future, frontier, generation: int, handle) -> None:
        if self._active_goal is not handle:
            return
        if self._finishing:
            self._active_goal = None
            return
        response = future.result()
        if self._navigation_pending and response is not None and response.status in (
                GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self._goals_preempted += 1
        elif response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
            self._goals_failed += 1
            self._blacklist.append((frontier.goal_x, frontier.goal_y))
        else:
            pose = self._map_pose()
            if pose is not None and math.dist(
                    (pose.x, pose.y), (frontier.goal_x, frontier.goal_y)) > 0.4:
                # The old FollowPath may finish before the BT's periodic
                # replan incorporates a replacement goal. Retry that target;
                # do not count the old path's endpoint as the new destination.
                self._ready_frontier = frontier
            else:
                self._goals_succeeded += 1
                self._visited.append((frontier.goal_x, frontier.goal_y,
                                      self._sim_seconds() + 10.0))
        self._active_goal = None
        self._active_frontier = None
        self._remaining_distance = None
        self._cancel_requested = False

    def _navigation_feedback(self, message, generation):
        if generation == self._active_generation:
            self._remaining_distance = max(0.0, message.feedback.distance_remaining)

    def _finish(self, reason: str, *, status: str = 'succeeded') -> None:
        if self._finishing:
            return
        self._finishing = True
        self._finish_state = 'stopping'
        self._finish_reason = reason
        self._finish_status = status
        self._finish_started_wall = time.monotonic()
        self._stationary_since = None
        self._ready_frontier = None
        self._publish_status('stopping')
        if self._active_goal is not None:
            self._cancel_futures.append(self._active_goal.cancel_goal_async())
        if self._active_spin is not None:
            self._cancel_futures.append(self._active_spin.cancel_goal_async())

    def _wall_tick(self):
        now = time.monotonic()
        if not self._finishing:
            if now - self._last_map_wall > 30.0:
                self._finish('map_updates_unavailable', status='incomplete')
            elif not self._initialized and now - self._started_wall > 60.0:
                self._finish('initialization_timeout', status='incomplete')
            elif self._navigation_pending and now - self._navigation_request_wall > 10.0:
                self._finish('navigation_acceptance_timeout', status='incomplete')
            elif (self._spin_pending or self._active_spin is not None) and now - self._spin_started_wall > 35.0:
                self._finish('recovery_action_timeout', status='incomplete')
            elif self._selection_waiting and now - self._selection_started_wall > 30.0:
                self._finish('planner_response_timeout', status='incomplete')
            return
        # Hold is deliberately a stationary results session, not manual navigation.
        # This also protects against a delayed action acceptance/cancel response.
        self._stop_pub.publish(Twist())
        if self._finish_state == 'stopping':
            acknowledged = not self._navigation_pending and not self._spin_pending
            for future in self._cancel_futures:
                if not future.done():
                    acknowledged = False
                else:
                    try:
                        response = future.result()
                        acknowledged &= response is not None and (
                            bool(response.goals_canceling) or (
                                self._active_goal is None and self._active_spin is None))
                    except Exception:
                        acknowledged = False
            stationary = (self._stationary_since is not None
                          and now - self._stationary_since >= 1.0
                          and now - self._last_odom_wall < 2.0)
            if not (acknowledged and stationary):
                if now - self._finish_started_wall < 8.0:
                    return
                self._warnings.append('Stop confirmation timed out; zero velocity remains enforced.')
                self._finish_status = 'incomplete'
                self._finish_reason = 'stop_confirmation_timeout'
            if now - self._last_map_wall > 30.0:
                self._warnings.append('Final map is stale; the last available snapshot was saved.')
                self._finish_status = 'incomplete'
            self._finished_sim_ns = self.get_clock().now().nanoseconds
            self._finish_state = 'saving'
            self._publish_status('saving')
            self._final_snapshot = deepcopy(self._map)
            # Never expose partially changing ROS messages to the worker.
            self._final_future = self._analysis_pool.submit(
                self._save_snapshot, self._final_snapshot,
                self._finish_reason, self._finish_status)
        elif self._finish_state == 'saving' and self._final_future.done():
            try:
                result = self._final_future.result()
            except Exception as error:
                result = self._result_metadata(self._finish_reason, 'save_failed')
                result.update(map_status='save_failed', map_filename='')
                result['warnings'].append(f'Could not save run: {error}')
            self._present_result(result)

    def _save_snapshot(self, snapshot, reason, status):
        result = self._result_metadata(reason, status)
        if snapshot is not None and self._alignment is not None:
            info = snapshot.info
            spec = GridSpec(info.width, info.height, info.resolution,
                            info.origin.position.x, info.origin.position.y,
                            _quaternion_yaw(info.origin.orientation))
            try:
                result.update(score_grid(
                    list(snapshot.data), spec, self._boxes, self._alignment,
                    (float(self.get_parameter('start_x').value),
                     float(self.get_parameter('start_y').value))))
            except Exception as error:
                result.update(status='incomplete', completion_reason='metrics_unavailable')
                result['warnings'].append(f'Final scoring failed: {error}')
        if result['status'] == 'succeeded' and (
                result.get('coverage_percent') is None
                or result['coverage_percent'] < result['target_coverage_percent']):
            result.update(status='incomplete', completion_reason='final_snapshot_below_target')
        if bool(self.get_parameter('record_results').value):
            try:
                result['map_filename'] = write_map(result['output_directory'], snapshot)
                result['map_status'] = 'saved_direct'
            except Exception as error:
                result.update(status='save_failed', map_status='save_failed', map_filename='')
                result['warnings'].append(f'Map save failed: {error}')
            try:
                write_report(result['output_directory'], result)
            except Exception as error:
                result['status'] = 'save_failed'
                result['warnings'].append(f'Report save failed: {error}')
        else:
            result.update(map_status='not_requested', map_filename='')
        return result

    def _result_metadata(self, reason, status):
        elapsed = 0.0
        if self._started_sim_ns is not None and self._finished_sim_ns is not None:
            elapsed = (self._finished_sim_ns - self._started_sim_ns) / 1e9
        return {
            'schema_version': 2,
            'seed': int(self.get_parameter('seed').value),
            'status': status,
            'completion_reason': reason,
            'started_at': self._started_at,
            'finished_at': datetime.now(timezone.utc).isoformat(),
            'exploration_time_seconds': round(elapsed, 3),
            'wall_time_seconds': round(time.monotonic() - self._started_wall, 3),
            'distance_traveled_m': round(self._distance, 3),
            'frontier_goals_attempted': self._goals_attempted,
            'frontier_goals_succeeded': self._goals_succeeded,
            'frontier_goals_failed': self._goals_failed,
            'frontier_goals_preempted': self._goals_preempted,
            'recovery_spins': self._recovery_spins,
            'recovery_spins_succeeded': self._recovery_spins_succeeded,
            'recovery_spins_failed': self._recovery_spins_failed,
            'completion_policy': 'target_coverage_and_no_reachable_frontiers',
            'target_coverage_percent': float(self.get_parameter('target_coverage').value),
            'output_directory': str(Path(str(self.get_parameter('output_directory').value)).expanduser().resolve()),
            'warnings': list(self._warnings),
            'layout_file': str(self.get_parameter('layout_file').value),
            'world_file': str(self.get_parameter('world_file').value),
        }

    def _present_result(self, result):
        self._finish_state = 'finished'
        self._metrics = result
        self.exit_code = 0 if result['status'] == 'succeeded' else 1
        self._publish_frontiers([])
        self._publish_status(result['status'])
        summary = summary_text(result)
        shutdown = self.get_parameter('finish_behavior').value == 'shutdown'
        ending = 'Closing the run.' if shutdown else 'Ctrl+C closes this results session.'
        self.get_logger().info('\n' + summary + '\nRobot stopped. ' + ending)
        if self._final_snapshot is not None:
            self._final_map_pub.publish(self._final_snapshot)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'exploration_summary'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 1.0
        if self._final_snapshot is not None:
            info = self._final_snapshot.info
            marker.pose.position.x = info.origin.position.x + info.width * info.resolution / 2
            marker.pose.position.y = info.origin.position.y - 2.0
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 1.0 if self.exit_code == 0 else 0.4
        marker.color.b = 0.3
        marker.color.a = 1.0
        # Long absolute filenames make a world-space label span the whole arena.
        marker.text = '\n'.join(summary.splitlines()[:8]) + '\nFiles and reload command: terminal / summary.md'
        self._summary_pub.publish(marker)
        if shutdown:
            self._shutdown_once()

    def _shutdown_once(self) -> None:
        self._shutdown_requested = True
        if rclpy.ok():
            self._analysis_pool.shutdown(wait=False, cancel_futures=True)
            self.destroy_node()
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ExplorationMission()
        rclpy.spin(node)
        code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        code = node.exit_code if node is not None and node._shutdown_requested else 0
    except Exception as exception:
        # A signal can invalidate a wait-set/publisher between the executor's
        # context check and its C++ call. Do not turn normal Ctrl+C into a failure.
        if not rclpy.ok():
            code = node.exit_code if node is not None and node._shutdown_requested else 0
        else:
            if node is not None:
                node.get_logger().error(f'Exploration mission failed: {type(exception).__name__}: {exception}')
            code = 1
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        if node is not None:
            node._analysis_pool.shutdown(wait=True, cancel_futures=True)
        if rclpy.ok():
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
