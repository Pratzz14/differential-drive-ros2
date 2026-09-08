"""Protect the shared ROS topics and isolate each Gazebo transport session."""

import atexit
import fcntl
import os
import time
import uuid

from launch import LaunchDescription
from launch.actions import (
    LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def _setup(context):
    domain = int(context.environment.get('ROS_DOMAIN_ID', '0'))
    lock_path = f'/tmp/differential_drive_robot_{os.getuid()}_domain_{domain}.lock'
    descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(descriptor)
        raise RuntimeError(
            f'A robot simulation is already running in ROS_DOMAIN_ID={domain}. '
            'Stop its launch with Ctrl+C before starting another.'
        )
    # Keep the lock until the launch process exits, including child shutdown.
    # Do not unlink it: another process may already have opened the same inode.
    atexit.register(os.close, descriptor)

    ros_context = Context()
    rclpy.init(args=[], context=ros_context, domain_id=domain,
               signal_handler_options=SignalHandlerOptions.NO)
    probe = rclpy.create_node('robot_launch_preflight', context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        conflicts = [topic for topic in ('/clock', '/odom', '/joint_states', '/scan')
                     if probe.count_publishers(topic)]
        stale_nodes = sorted({name for name, namespace in probe.get_node_names_and_namespaces()
                              if namespace == '/' and name in {
                                  'amcl', 'ekf_filter_node', 'robot_state_publisher',
                                  'controller_server', 'planner_server', 'bt_navigator',
                                  'lifecycle_manager_localization',
                                  'lifecycle_manager_navigation',
                              }})
        if conflicts or stale_nodes:
            raise RuntimeError(
                f'ROS_DOMAIN_ID={domain} still contains simulation publishers '
                f'{conflicts} or nodes {stale_nodes}. Stop the previous launch '
                'before restarting; restarting the ROS daemon does not stop nodes.'
            )
    finally:
        executor.shutdown()
        probe.destroy_node()
        ros_context.shutdown()

    partition = f'differential_drive_{os.getuid()}_{uuid.uuid4().hex}'
    return [
        SetEnvironmentVariable('GZ_PARTITION', partition),
        LogInfo(msg=f'Robot session: ROS_DOMAIN_ID={domain}, GZ_PARTITION={partition}'),
    ]


def _stop_on_exit(event, context):
    if context.is_shutdown:
        return []
    is_node = isinstance(event.action, Node)
    # launch_ros may retain <node_namespace_unspecified> in node_name.
    name = event.action.node_name.rsplit('/', 1)[-1] if is_node else ''
    expected_exit = name in {
        'spawn_differential_drive_robot', 'navigation_mission',
        'navigation_rviz', 'rviz2',
    }
    if event.returncode != 0 or (is_node and not expected_exit):
        return [Shutdown(reason=(
            f'{event.process_name} exited with code {event.returncode}; '
            'stopping the simulation to avoid a partially running stack.'
        ))]
    return []


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_setup),
        RegisterEventHandler(OnProcessExit(on_exit=_stop_on_exit)),
    ])
