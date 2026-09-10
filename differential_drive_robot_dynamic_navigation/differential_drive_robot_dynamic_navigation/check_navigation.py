#!/usr/bin/env python3
"""Read-only runtime check: ros2 run ... check_navigation.py --ros-args -p duration:=15.0."""
import json
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class NavigationCheck(Node):
    def __init__(self):
        super().__init__('dynamic_navigation_check')
        self.declare_parameter('duration', 15.0)
        self.messages = {}
        self.counts = {}
        self.received = {}
        self.backward_jumps = 0
        self.clock_start = None
        self.clock_last = None
        self.scan_first = None
        self.scan_last = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, msg_type, qos in [
            ('/clock', Clock, qos_profile_sensor_data),
            ('/scan', LaserScan, qos_profile_sensor_data),
            ('/joint_states', JointState, qos_profile_sensor_data),
            ('/odometry/filtered', Odometry, qos_profile_sensor_data),
            ('/map', OccupancyGrid, latched),
            ('/robot_description', String, latched),
            ('/dynamic_obstacles/predicted_cloud_odom', PointCloud2, qos_profile_sensor_data),
        ]:
            self.create_subscription(msg_type, topic, lambda msg, key=topic: self.receive(key, msg), qos)

    def receive(self, topic, message):
        self.messages[topic] = message
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.received[topic] = time.monotonic()
        if topic == '/clock':
            stamp = message.clock.sec + message.clock.nanosec / 1e9
            if self.clock_last is not None and stamp < self.clock_last:
                self.backward_jumps += 1
            if self.clock_start is None:
                self.clock_start = stamp
            self.clock_last = stamp
        elif topic == '/scan':
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if self.scan_first is None:
                self.scan_first = stamp
            self.scan_last = stamp

    def report(self):
        required = ['/clock', '/scan', '/joint_states', '/odometry/filtered', '/map', '/robot_description']
        issues = [f'No messages: {topic}' for topic in required if topic not in self.messages]
        for topic in required[:4]:
            if topic in self.received and time.monotonic() - self.received[topic] > 2.0:
                issues.append(f'Stale stream: {topic}')
        publishers = self.count_publishers('/clock')
        if publishers != 1:
            issues.append(f'Expected exactly one /clock publisher, found {publishers}')
        if self.backward_jumps:
            issues.append(f'{self.backward_jumps} backward clock jumps')
        if self.clock_start == self.clock_last:
            issues.append('Simulation clock is not advancing (paused or no clock)')
        scan = self.messages.get('/scan')
        if scan and not any(math.isfinite(r) and scan.range_min <= r <= scan.range_max for r in scan.ranges):
            issues.append('Scan contains no valid returns')
        frames = {}
        for frame in ['base_link', 'lidar_link', 'left_wheel_link', 'right_wheel_link']:
            try:
                tf = self.buffer.lookup_transform('map', frame, Time())
                p = tf.transform.translation
                frames[frame] = [round(p.x, 3), round(p.y, 3)]
            except TransformException as exc:
                issues.append(f'map -> {frame}: {exc}')
        if scan and not self.buffer.can_transform('map', scan.header.frame_id, Time.from_msg(scan.header.stamp)):
            issues.append('No map transform at the latest laser timestamp')
        elapsed = (self.scan_last or 0) - (self.scan_first or 0)
        return {'healthy': not issues, 'issues': issues, 'clock_publishers': publishers,
                'backward_clock_jumps': self.backward_jumps, 'messages': self.counts,
                'scan_hz_sim_time': round((self.counts.get('/scan', 1) - 1) / elapsed, 2) if elapsed > 0 else 0,
                'map_frame_positions': frames}


def main():
    rclpy.init()
    node = NavigationCheck()
    try:
        end = time.monotonic() + float(node.get_parameter('duration').value)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        result = node.report()
        print(json.dumps(result, indent=2), flush=True)
        return 0 if result['healthy'] else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
