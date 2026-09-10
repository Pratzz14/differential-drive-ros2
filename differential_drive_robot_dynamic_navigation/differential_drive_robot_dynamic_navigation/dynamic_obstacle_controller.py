#!/usr/bin/env python3
"""Drive generated boxes between their patrol endpoints through Gazebo bridges."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger


def _velocity_components(ux: float, uy: float, command_speed: float) -> tuple[float, float]:
    return ux * command_speed, uy * command_speed


def _body_velocity(dx: float, dy: float, speed: float, yaw: float) -> tuple[float, float]:
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0, 0.0
    vx, vy = dx / length * speed, dy / length * speed
    return math.cos(yaw) * vx + math.sin(yaw) * vy, -math.sin(yaw) * vx + math.cos(yaw) * vy


class DynamicObstacleController(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_obstacle_controller')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('acceleration', 0.40)
        self.declare_parameter('endpoint_tolerance', 0.08)
        self.declare_parameter('dwell_seconds', 0.5)
        self._active = False
        self._clock_start = None
        self._states = []
        layout_file = Path(str(self.get_parameter('layout_file').value))
        layout = json.loads(layout_file.read_text(encoding='utf-8'))
        for item in layout.get('dynamic_obstacles', []):
            state = {
                'name': item['name'], 'x': item['x'], 'y': item['y'],
                'end_x': item['end_x'], 'end_y': item['end_y'],
                'speed': item['speed'], 'direction': 1, 'dwell_until': 0.0,
                'pose': None, 'pose_received': 0.0, 'command_speed': 0.0,
            }
            state['dx'] = item['end_x'] - item['x']
            state['dy'] = item['end_y'] - item['y']
            length = math.hypot(state['dx'], state['dy']) or 1.0
            state['ux'], state['uy'] = state['dx'] / length, state['dy'] / length
            state['publisher'] = self.create_publisher(
                Twist, f"/dynamic_obstacles/obstacle_{len(self._states)}/cmd_vel", 10)
            self._states.append(state)
            self.create_subscription(
                Pose, f"/dynamic_obstacles/obstacle_{len(self._states) - 1}/pose",
                lambda message, s=state: self._pose_callback(s, message), 10)
        self.create_service(Trigger, '/dynamic_obstacles/start', self._start_callback)
        self.create_service(Trigger, '/dynamic_obstacles/stop', self._stop_callback)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f'Loaded {len(self._states)} moving obstacles')

    def _pose_callback(self, state, message: Pose) -> None:
        state['pose'] = message
        state['pose_received'] = time.monotonic()

    def _start_callback(self, _request, response):
        self._active = True
        self._clock_start = self.get_clock().now().nanoseconds / 1e9
        for state in self._states:
            state['direction'] = 1
            state['dwell_until'] = 0.0
            state['command_speed'] = 0.0
        response.success = True
        response.message = 'Dynamic obstacle patrols started'
        self.get_logger().info(response.message)
        return response

    def _publish_zero(self, state) -> None:
        state['publisher'].publish(Twist())

    def _stop_callback(self, _request, response):
        self.stop()
        response.success = True
        response.message = 'Patrols stopped'
        return response

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        for state in self._states:
            if (not self._active or state['pose'] is None or
                    time.monotonic() - state['pose_received'] > 1.0):
                state['command_speed'] = 0.0
                self._publish_zero(state)
                continue
            pose = state['pose']
            target_x = state['end_x'] if state['direction'] > 0 else state['x']
            target_y = state['end_y'] if state['direction'] > 0 else state['y']
            remaining = math.hypot(target_x - pose.position.x, target_y - pose.position.y)
            if now < state['dwell_until']:
                self._publish_zero(state)
                continue
            if remaining <= float(self.get_parameter('endpoint_tolerance').value):
                state['direction'] *= -1
                state['dwell_until'] = now + float(self.get_parameter('dwell_seconds').value)
                state['command_speed'] = 0.0
                self._publish_zero(state)
                continue
            # Brake before endpoints and steer toward the actual target. A box
            # displaced/rotated by contact must not keep driving past its patrol.
            desired = min(state['speed'], math.sqrt(2.0 * float(self.get_parameter('acceleration').value) *
                                                    max(0.0, remaining - 0.04)))
            acceleration = float(self.get_parameter('acceleration').value) * 0.05
            state['command_speed'] += max(-acceleration, min(acceleration, desired - state['command_speed']))
            command = Twist()
            q = pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            command.linear.x, command.linear.y = _body_velocity(
                target_x - pose.position.x, target_y - pose.position.y, state['command_speed'], yaw)
            state['publisher'].publish(command)

    def stop(self) -> None:
        self._active = False
        for state in self._states:
            self._publish_zero(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
