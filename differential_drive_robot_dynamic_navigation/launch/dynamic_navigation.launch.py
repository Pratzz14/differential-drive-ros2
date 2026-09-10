#!/usr/bin/env python3
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.logging import get_logger
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, Shutdown, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from differential_drive_robot_dynamic_navigation.session_process import SessionNode as Node, SessionProcess
from differential_drive_robot_dynamic_navigation.shutdown import orderly_shutdown


def _gazebo_exit(event, context):
    if context.is_shutdown:
        return []
    return [Shutdown(reason=f'{event.process_name} exited ({event.returncode}); stopping the robot session')]


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    share = get_package_share_directory('differential_drive_robot_dynamic_navigation')
    description = get_package_share_directory('differential_drive_robot_description')
    seed_text = LaunchConfiguration('seed').perform(context).strip()
    seed = None if seed_text in ('', '-1', 'random') else int(seed_text)
    robot_speed = float(LaunchConfiguration('robot_speed').perform(context))
    min_speed = float(LaunchConfiguration('obstacle_min_speed').perform(context))
    max_speed = float(LaunchConfiguration('obstacle_max_speed').perform(context))
    if not 0.10 <= robot_speed <= 0.50:
        raise ValueError('robot_speed must be between 0.10 and 0.50 m/s')
    if not 0 < min_speed <= max_speed <= 0.6 * robot_speed:
        raise ValueError('Obstacle speeds must be positive, ordered and at most 60% of robot_speed')
    from differential_drive_robot_dynamic_navigation.obstacle_generator import generate_world
    _, world, layout = generate_world(
        Path(share) / 'worlds' / 'dynamic_arena.sdf.in',
        LaunchConfiguration('generated_world_directory').perform(context), seed,
        int(LaunchConfiguration('static_obstacle_count').perform(context)),
        int(LaunchConfiguration('dynamic_obstacle_count').perform(context)),
        min_speed, max_speed,
    )
    urdf = Path(description) / 'urdf' / 'differential_drive_robot.urdf'
    robot_description = urdf.read_text(encoding='utf-8')
    resource_path = os.pathsep.join(filter(None, [str(Path(description).parent), description,
                                                 context.environment.get('GZ_SIM_RESOURCE_PATH', '')]))
    nav2 = Path(share) / 'config' / 'nav2.yaml'
    bridge = Path(share) / 'config' / 'navigation_bridge.yaml'
    ekf = Path(share) / 'config' / 'ekf.yaml'
    collision = Path(share) / 'config' / 'collision_monitor.yaml'
    mode = LaunchConfiguration('avoidance_mode').perform(context)
    if mode not in ('predictive', 'reactive'):
        raise ValueError('avoidance_mode must be predictive or reactive')
    tracker_condition = IfCondition('true' if mode == 'predictive' else 'false')
    common = [str(nav2), {'use_sim_time': True,
                         'FollowPath.desired_linear_vel': robot_speed,
                         'default_nav_to_pose_bt_xml': str(Path(share) / 'behavior_trees' / 'navigate.xml')}]
    shutdown_started = False

    def stop_session(shutdown_context):
        nonlocal shutdown_started
        if not shutdown_started:
            shutdown_started = True
            try:
                orderly_shutdown(shutdown_context.environment)
            except Exception as error:
                # A failed service/context must not prevent launch's remaining
                # handlers from delivering signals and reaping all children.
                get_logger('dynamic_navigation_shutdown').warning(
                    f'Orderly shutdown failed: {error}; continuing process shutdown')
        return []

    return [
        # Register before child process handlers: services run while ROS and
        # simulation time are alive, then launch signals the isolated children.
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=stop_session)])),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SessionProcess(cmd=['gz', 'sim', '-s', '-r', str(world)], name='gazebo_server',
                       output='screen', on_exit=_gazebo_exit),
        SessionProcess(cmd=['gz', 'sim', '-g'], name='gazebo_gui', output='screen',
                       condition=UnlessCondition(LaunchConfiguration('headless')), on_exit=_gazebo_exit),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             output='screen', parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='dynamic_navigation_bridge',
             output='screen', parameters=[{'config_file': str(bridge), 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', name='spawn_differential_drive_robot', output='screen',
             arguments=['-world', 'dynamic_arena', '-file', str(urdf), '-name', 'differential_drive_robot',
                        '-x', '-5.0', '-y', '0.0', '-z', '0.08']),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[str(ekf), {'use_sim_time': True}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[str(nav2), {'use_sim_time': True, 'yaml_filename': str(Path(share) / 'maps' / 'navigation_map.yaml')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen', parameters=common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen',
             parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        TimerAction(period=5.0, actions=[Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'use_sim_time': True, 'autostart': True,
                                           'node_names': ['controller_server', 'smoother_server', 'planner_server',
                                                          'behavior_server', 'velocity_smoother',
                                                          'collision_monitor', 'bt_navigator']}])]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen',
             parameters=[str(collision), {'use_sim_time': True}],
             remappings=[('cmd_vel_smoothed', '/cmd_vel_smoothed'), ('cmd_vel', '/cmd_vel')]),
        Node(package='differential_drive_robot_dynamic_navigation', executable='dynamic_obstacle_controller.py',
             name='dynamic_obstacle_controller', output='screen', parameters=[{'use_sim_time': True, 'layout_file': str(layout)}]),
        Node(package='differential_drive_robot_dynamic_navigation', executable='motion_tracker.py', name='motion_tracker',
             output='screen', parameters=[{'use_sim_time': True, 'prediction_horizon': 1.0,
                                         'max_obstacle_speed': max(0.25, 1.8 * max_speed)}], condition=tracker_condition),
        Node(package='differential_drive_robot_dynamic_navigation', executable='mission_runner.py', name='dynamic_navigation_mission',
             output='screen', parameters=[{'use_sim_time': True, 'layout_file': str(layout),
                                           'auto_goal': _bool(context, 'auto_goal'),
                                           'goal_x': float(LaunchConfiguration('goal_x').perform(context)),
                                           'goal_y': float(LaunchConfiguration('goal_y').perform(context)),
                                           'goal_yaw': float(LaunchConfiguration('goal_yaw').perform(context)),
                                           'results_directory': LaunchConfiguration('results_directory').perform(context),
                                           'mission_timeout': float(LaunchConfiguration('mission_timeout').perform(context)),}]),
        TimerAction(period=10.0, actions=[Node(package='rviz2', executable='rviz2', name='dynamic_navigation_rviz', output='screen',
             arguments=['-d', str(Path(share) / 'config' / 'navigation.rviz')], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')))]),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Do not block lifecycle/sensor executor threads on DDS writes during
        # GUI startup. Respect an explicit user-selected publication mode.
        SetEnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE',
                               EnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE', default_value='ASYNCHRONOUS')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('differential_drive_robot_simulation'),
                'launch', 'session.launch.py',
            )
        )),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('robot_speed', default_value='0.45'),
        DeclareLaunchArgument('obstacle_min_speed', default_value='0.10'),
        DeclareLaunchArgument('obstacle_max_speed', default_value='0.22'),
        DeclareLaunchArgument('avoidance_mode', default_value='predictive'),
        DeclareLaunchArgument('static_obstacle_count', default_value='2'),
        DeclareLaunchArgument('dynamic_obstacle_count', default_value='4'),
        DeclareLaunchArgument('auto_goal', default_value='true'),
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('mission_timeout', default_value='180.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('generated_world_directory', default_value='/tmp/differential_drive_dynamic/worlds'),
        DeclareLaunchArgument('results_directory', default_value='/tmp/differential_drive_dynamic/results'),
        OpaqueFunction(function=_setup),
    ])
