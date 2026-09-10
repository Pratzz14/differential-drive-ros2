#!/usr/bin/env python3
"""Estimate moving obstacle velocity from successive LiDAR scans."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 0
    last_time: float = 0.0
    observations: list = field(default_factory=list)


def _active_tracks(tracks: list[Track], stamp: float, timeout: float) -> list[Track]:
    return [track for track in tracks if 0.0 <= stamp - track.last_time <= timeout]


def _update_velocity(track: Track, stamp: float, x: float, y: float) -> None:
    """Fit motion over a window, not the noisy difference of two scan centroids."""
    track.observations.append((stamp, x, y))
    track.observations = [p for p in track.observations if stamp - p[0] <= 0.8]
    if len(track.observations) < 4 or stamp - track.observations[0][0] < 0.4:
        track.vx = track.vy = 0.0
        return
    count = len(track.observations)
    mt, mx, my = (sum(p[i] for p in track.observations) / count for i in range(3))
    variance = sum((p[0] - mt) ** 2 for p in track.observations)
    track.vx = sum((t - mt) * (px - mx) for t, px, _ in track.observations) / variance
    track.vy = sum((t - mt) * (py - my) for t, _, py in track.observations) / variance


def _transform_xy(x: float, y: float, transform) -> tuple[float, float]:
    q = transform.transform.rotation
    tx, ty = transform.transform.translation.x, transform.transform.translation.y
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    s = 2.0 * (q.w * q.z + q.x * q.y)
    return tx + c * x - s * y, ty + s * x + c * y


def _near_static_map(x: float, y: float, grid: OccupancyGrid, margin: float = 0.18) -> bool:
    """Reject known walls even when occlusion makes their visible cluster small."""
    info = grid.info
    if info.resolution <= 0 or len(grid.data) != info.width * info.height:
        return False
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    dx, dy = x - info.origin.position.x, y - info.origin.position.y
    gx = (math.cos(yaw) * dx + math.sin(yaw) * dy) / info.resolution
    gy = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / info.resolution
    radius = math.ceil(margin / info.resolution) + 1
    for iy in range(max(0, math.floor(gy) - radius), min(info.height, math.floor(gy) + radius + 1)):
        for ix in range(max(0, math.floor(gx) - radius), min(info.width, math.floor(gx) + radius + 1)):
            # Distance to occupied cell rectangle, not only its centre.
            distance = math.hypot(max(ix-gx, 0, gx-ix-1), max(iy-gy, 0, gy-iy-1)) * info.resolution
            if grid.data[iy * info.width + ix] >= 65 and distance <= margin:
                return True
    return False


class MotionTracker(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_obstacle_motion_tracker')
        self.declare_parameter('prediction_horizon', 2.0)
        self.declare_parameter('prediction_step', 0.25)
        self.declare_parameter('moving_speed_threshold', 0.04)
        self.declare_parameter('track_timeout', 0.5)
        self.declare_parameter('cluster_jump', 0.65)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('max_cluster_extent', 0.9)
        self.declare_parameter('max_obstacle_speed', 0.40)
        self.static_map = None
        self.create_subscription(OccupancyGrid, '/map', self._map_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tracks: list[Track] = []
        self.next_id = 0
        self.pending_scan = None
        self.last_scan_stamp = None
        self.odom_pub = self.create_publisher(PointCloud2, '/dynamic_obstacles/predicted_cloud_odom', 10)
        self.map_pub = self.create_publisher(PointCloud2, '/dynamic_obstacles/predicted_cloud_map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dynamic_obstacles/tracks', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)
        self.create_timer(0.02, self._process_pending_scan)
        self.create_timer(0.1, self._expire_tracks)

    def _map_callback(self, grid):
        self.static_map = grid

    def _clusters(self, scan: LaserScan) -> list[tuple[float, float]]:
        points = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < max(scan.range_min, 0.12) or distance > scan.range_max:
                points.append(None)
                continue
            angle = scan.angle_min + index * scan.angle_increment
            points.append((distance * math.cos(angle), distance * math.sin(angle)))
        clusters = []
        current = []
        jump = float(self.get_parameter('cluster_jump').value)
        minimum = int(self.get_parameter('min_cluster_points').value)
        for point in points + [None]:
            if point is None or (current and math.dist(current[-1], point) > jump):
                extent = max((math.dist(current[0], p) for p in current), default=0.0)
                if len(current) >= minimum and extent <= float(self.get_parameter('max_cluster_extent').value):
                    clusters.append((sum(x for x, _ in current) / len(current),
                                     sum(y for _, y in current) / len(current)))
                current = []
            if point is not None:
                current.append(point)
        return clusters

    def _scan_callback(self, scan: LaserScan) -> None:
        self.pending_scan = scan

    def _expire_tracks(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        timeout = float(self.get_parameter('track_timeout').value)
        active = _active_tracks(self.tracks, now, timeout)
        if len(active) != len(self.tracks):
            self.tracks = active
            self._publish(now)

    def _process_pending_scan(self) -> None:
        scan = self.pending_scan
        if scan is None:
            return
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        try:
            # Retry on a timer: blocking in a single-threaded scan callback
            # prevents the missing TF from being received in the first place.
            transform = self.tf_buffer.lookup_transform('odom', scan.header.frame_id,
                                                         Time.from_msg(scan.header.stamp))
            if self.static_map is None:
                return
            map_transform = self.tf_buffer.lookup_transform(self.static_map.header.frame_id, 'odom',
                                                             Time.from_msg(scan.header.stamp))
        except TransformException:
            return
        self.pending_scan = None
        if self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            self.tracks.clear()
        self.last_scan_stamp = stamp
        timeout = float(self.get_parameter('track_timeout').value)
        self.tracks = _active_tracks(self.tracks, stamp, timeout)
        observed = [_transform_xy(*point, transform) for point in self._clusters(scan)]
        observed = [point for point in observed if not _near_static_map(
            *_transform_xy(*point, map_transform), self.static_map)]
        self.tracks = [track for track in self.tracks if not _near_static_map(
            *_transform_xy(track.x, track.y, map_transform), self.static_map)]
        unmatched = set(range(len(observed)))
        for track in self.tracks:
            age = max(0.01, stamp - track.last_time)
            predicted = (track.x + track.vx * age, track.y + track.vy * age)
            gate = 0.30 + 0.25 * age
            candidates = [(math.dist(predicted, observed[i]), i) for i in unmatched]
            if not candidates:
                continue
            distance, index = min(candidates)
            if distance > gate:
                continue
            ox, oy = observed[index]
            _update_velocity(track, stamp, ox, oy)
            track.x, track.y = ox, oy
            track.hits += 1
            track.last_time = stamp
            unmatched.remove(index)
        for index in unmatched:
            self.tracks.append(Track(self.next_id, observed[index][0], observed[index][1], hits=1,
                                     last_time=stamp, observations=[(stamp, *observed[index])]))
            self.next_id += 1
        timeout = float(self.get_parameter('track_timeout').value)
        self.tracks = _active_tracks(self.tracks, stamp, timeout)
        self._publish(stamp)

    def _prediction_points(self) -> list[tuple[float, float]]:
        horizon = float(self.get_parameter('prediction_horizon').value)
        step = float(self.get_parameter('prediction_step').value)
        threshold = float(self.get_parameter('moving_speed_threshold').value)
        points = []
        for track in self.tracks:
            speed = math.hypot(track.vx, track.vy)
            if track.hits < 6 or not threshold <= speed <= float(self.get_parameter('max_obstacle_speed').value):
                continue
            t = step
            while t <= horizon + 1e-6:
                cx, cy = track.x + track.vx * t, track.y + track.vy * t
                for angle in range(0, 360, 45):
                    radius = 0.20
                    points.append((cx + radius * math.cos(math.radians(angle)),
                                   cy + radius * math.sin(math.radians(angle))))
                t += step
        return points

    def _publish(self, stamp: float) -> None:
        points = self._prediction_points()
        now = self.get_clock().now().to_msg()
        odom_header = Header(stamp=now, frame_id='odom')
        self.odom_pub.publish(point_cloud2.create_cloud_xyz32(odom_header, [(x, y, 0.12) for x, y in points]))
        map_points = []
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', Time())
            map_points = [_transform_xy(x, y, transform) for x, y in points]
        except TransformException:
            pass
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=now, frame_id='map'),
                                                              [(x, y, 0.12) for x, y in map_points]))
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for track in self.tracks:
            marker = Marker(header=Header(stamp=now, frame_id='odom'), ns='dynamic_tracks', id=track.track_id,
                            type=Marker.ARROW, action=Marker.ADD)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.03, 0.07, 0.07
            marker.lifetime = Duration(seconds=float(self.get_parameter('track_timeout').value)).to_msg()
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 1.0, 0.9
            marker.points = [Point(x=track.x, y=track.y, z=0.25),
                            Point(x=track.x + track.vx, y=track.y + track.vy, z=0.25)]
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A SIGINT can invalidate a subscription while the executor takes a
        # message. Do not report that as a tracker crash after context shutdown.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
