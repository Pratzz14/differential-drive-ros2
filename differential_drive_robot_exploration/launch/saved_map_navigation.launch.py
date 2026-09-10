#!/usr/bin/env python3
"""Navigate the generated arena using a map saved by exploration."""
# flake8: noqa

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORLD_START_X = -5.0
WORLD_START_Y = 0.0
WORLD_START_Z = 0.08
WORLD_START_YAW = 0.0


def _enabled(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _shutdown_once(event, context):
    if context.is_shutdown or getattr(context, '_robot_shutdown_requested', False):
        return []
    context._robot_shutdown_requested = True
    return [Shutdown(reason='Gazebo exited')]


def _launch_setup(context):
    navigation_share = get_package_share_directory('differential_drive_robot_navigation')
    description_share = get_package_share_directory('differential_drive_robot_description')

    map_file = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    world_file = Path(LaunchConfiguration('world').perform(context)).expanduser().resolve()
    if not map_file.is_file():
        raise RuntimeError(f'Saved occupancy map does not exist: {map_file}')
    if not world_file.is_file():
        raise RuntimeError(f'Matching generated Gazebo world does not exist: {world_file}')

    urdf_file = os.path.join(description_share, 'urdf', 'differential_drive_robot.urdf')
    robot_description = Path(urdf_file).read_text(encoding='utf-8')
    resource_roots = os.pathsep.join([os.path.dirname(description_share), description_share])
    headless = _enabled(LaunchConfiguration('headless').perform(context))

    nav2_parameters = os.path.join(navigation_share, 'config', 'nav2.yaml')
    ekf_parameters = os.path.join(navigation_share, 'config', 'ekf.yaml')
    bridge_parameters = os.path.join(navigation_share, 'config', 'navigation_bridge.yaml')
    rviz_file = os.path.join(get_package_share_directory('differential_drive_robot_exploration'),
                             'config', 'exploration.rviz')
    common = [nav2_parameters, {'use_sim_time': True}]

    navigation_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             output='screen', parameters=common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[{
                 'use_sim_time': True,
                 'autostart': True,
                 'node_names': [
                     'controller_server', 'smoother_server', 'planner_server',
                     'behavior_server', 'bt_navigator', 'velocity_smoother',
                 ],
             }]),
    ]

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_roots),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', *(['-s'] if headless else []), str(world_file)],
            output='screen',
            on_exit=_shutdown_once,
        ),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='saved_map_ros_gz_bridge', output='screen',
             parameters=[{'config_file': bridge_parameters, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', name='spawn_differential_drive_robot',
             output='screen', arguments=[
                 '-world', 'navigation_arena', '-file', urdf_file,
                 '-name', 'differential_drive_robot',
                 '-x', str(WORLD_START_X), '-y', str(WORLD_START_Y),
                 '-z', str(WORLD_START_Z), '-Y', str(WORLD_START_YAW),
             ]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[ekf_parameters, {'use_sim_time': True}],
             remappings=[('odometry/filtered', '/odometry/filtered')]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[
                 nav2_parameters, {'yaml_filename': str(map_file), 'use_sim_time': True},
             ]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[nav2_parameters, {
                 'use_sim_time': True,
                 'set_initial_pose': True,
                 'initial_pose.x': ParameterValue(
                     LaunchConfiguration('initial_x'), value_type=float
                 ),
                 'initial_pose.y': ParameterValue(
                     LaunchConfiguration('initial_y'), value_type=float
                 ),
                 'initial_pose.z': 0.0,
                 'initial_pose.yaw': ParameterValue(
                     LaunchConfiguration('initial_yaw'), value_type=float
                 ),
             }]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[{
                 'use_sim_time': True,
                 'autostart': True,
                 'node_names': ['map_server', 'amcl'],
             }]),
        *navigation_nodes,
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             output='screen', arguments=['-d', rviz_file],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('differential_drive_robot_simulation'),
            'launch', 'session.launch.py',
        ))),
        DeclareLaunchArgument(
            'map',
            description='Absolute path to the map.yaml saved by exploration.',
        ),
        DeclareLaunchArgument(
            'world',
            description='Absolute path to the matching navigation_arena.sdf artifact.',
        ),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
