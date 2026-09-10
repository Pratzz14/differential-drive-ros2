#!/usr/bin/env python3
"""Start patrols, send a goal, and record a compact reproducible result."""
from __future__ import annotations
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class MissionRunner(Node):
    def __init__(self):
        super().__init__('dynamic_navigation_mission')
        for name, default in [('layout_file', ''), ('results_directory', '/tmp/differential_drive_dynamic/results'),
                              ('mission_timeout', 180.0), ('goal_x', 5.0), ('goal_y', 0.0), ('goal_yaw', 0.0)]:
            self.declare_parameter(name, default)
        self.declare_parameter('auto_goal', True)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.start_client = self.create_client(Trigger, '/dynamic_obstacles/start')
        self.bt_state_client = self.create_client(GetState, '/bt_navigator/get_state')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_scan = None
        self.last_progress_log = 0.0
        self.create_subscription(LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)

    def _scan_callback(self, scan):
        self.last_scan = scan

    def _feedback(self, message):
        now = time.monotonic()
        if now - self.last_progress_log >= 5.0:
            self.last_progress_log = now
            feedback = message.feedback
            self.get_logger().info(
                f'Navigation: {feedback.distance_remaining:.2f} m remaining; '
                f'{feedback.number_of_recoveries} recoveries')

    def _sensors_ready(self):
        if self.last_scan is None or not any(math.isfinite(r) for r in self.last_scan.ranges):
            return False
        stamp = Time.from_msg(self.last_scan.header.stamp)
        age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
        return (0.0 <= age < 0.5 and
                self.tf_buffer.can_transform('map', self.last_scan.header.frame_id, stamp) and
                self.tf_buffer.can_transform('map', 'base_link', stamp))

    def wait_for(self, predicate, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                return True
        return False

    def run(self):
        if not self.wait_for(self.client.server_is_ready, 90.0):
            raise RuntimeError('NavigateToPose action server unavailable')
        state_future = None
        def navigator_active():
            nonlocal state_future
            if state_future is None and self.bt_state_client.service_is_ready():
                state_future = self.bt_state_client.call_async(GetState.Request())
            if state_future is None or not state_future.done():
                return False
            active = state_future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            if not active:
                state_future = None
            return active
        if not self.wait_for(navigator_active, 90.0):
            raise RuntimeError('Nav2 navigator did not become active')
        if not self.wait_for(self._sensors_ready, 60.0):
            raise RuntimeError('No fresh LiDAR scan with map/robot TF; check Gazebo sensors and localization')
        if not self.wait_for(self.start_client.service_is_ready, 90.0):
            raise RuntimeError('Dynamic obstacle start service unavailable')
        request = self.start_client.call_async(Trigger.Request())
        if not self.wait_for(request.done, 5.0) or not request.result().success:
            raise RuntimeError('Dynamic obstacle patrols did not start')
        self.get_logger().info('Nav2 is ready; dynamic obstacle patrols started')
        if not bool(self.get_parameter('auto_goal').value):
            self.get_logger().info('Ready for an RViz 2D Goal Pose')
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.pose.pose.position.y = float(self.get_parameter('goal_y').value)
        yaw = float(self.get_parameter('goal_yaw').value)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        sent = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        if not self.wait_for(sent.done, 15.0) or not sent.result().accepted:
            raise RuntimeError('Navigation goal rejected')
        goal_handle = sent.result()
        result_future = goal_handle.get_result_async()
        started = time.monotonic()
        completed = self.wait_for(result_future.done, float(self.get_parameter('mission_timeout').value))
        status = 'timed_out'
        if completed:
            status_code = result_future.result().status
            status = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                      GoalStatus.STATUS_CANCELED: 'canceled'}.get(status_code, str(status_code))
        else:
            cancel_future = goal_handle.cancel_goal_async()
            self.wait_for(cancel_future.done, 5.0)
        result = {'status': status, 'elapsed_seconds': round(time.monotonic() - started, 3),
                  'finished_at': datetime.now(timezone.utc).isoformat(),
                  'layout_file': str(self.get_parameter('layout_file').value)}
        directory = Path(str(self.get_parameter('results_directory').value))
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"trial_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}.json"
        path.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        self._report_result(status, path)

    def _report_result(self, status, path):
        log = self.get_logger().info if status in ('succeeded', 'canceled') else self.get_logger().error
        log(f'Trial {status}; result: {path}')
        if status == 'canceled':
            self.get_logger().info('Navigation goal was canceled; no goal remains active.')
        elif status != 'succeeded':
            self.get_logger().error('Goal is no longer active. Inspect the trial/logs; send a new RViz goal after resolving the obstruction.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
