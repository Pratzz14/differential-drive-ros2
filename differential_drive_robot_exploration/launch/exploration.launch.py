#!/usr/bin/env python3
# flake8: noqa

import os
from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from differential_drive_robot_navigation.obstacle_generator import generate_world
from differential_drive_robot_exploration.run_output import create_run_directory


START = (-5.0, 0.0)


def _enabled(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _shutdown_once(event, context):
    if context.is_shutdown or getattr(context, '_robot_shutdown_requested', False):
        return []
    context._robot_shutdown_requested = True
    return [Shutdown(reason=f'{event.process_name} exited')]


def _launch_setup(context):
    package_share = get_package_share_directory('differential_drive_robot_exploration')
    navigation_share = get_package_share_directory('differential_drive_robot_navigation')
    simulation_share = get_package_share_directory('differential_drive_robot_simulation')
    description_share = get_package_share_directory('differential_drive_robot_description')

    seed_text = LaunchConfiguration('seed').perform(context)
    seed = None if seed_text.strip() in ('', '-1', 'random') else int(seed_text)
    count = int(LaunchConfiguration('obstacle_count').perform(context))
    layout, world_file, layout_file = generate_world(
        Path(simulation_share) / 'worlds' / 'navigation_arena.sdf.in',
        LaunchConfiguration('generated_world_directory').perform(context),
        seed, count, start=START, goal=(5.0, 0.0),
    )
    output_root = create_run_directory(LaunchConfiguration('output_directory').perform(context), layout.seed)
    shutil.copy2(world_file, output_root / 'navigation_arena.sdf')
    shutil.copy2(layout_file, output_root / 'layout.json')
    urdf_file = os.path.join(description_share, 'urdf', 'differential_drive_robot.urdf')
    robot_description = Path(urdf_file).read_text(encoding='utf-8')
    resource_roots = os.pathsep.join([os.path.dirname(description_share), description_share])
    headless = _enabled(LaunchConfiguration('headless').perform(context))

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', *(['-s'] if headless else []), str(world_file)],
        output='screen', on_exit=_shutdown_once,
    )
    common = [os.path.join(navigation_share, 'config', 'nav2.yaml'), {'use_sim_time': True}]
    nav_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[*common, os.path.join(
                 package_share, 'config', 'exploration_controller.yaml')],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             output='screen', parameters=common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_exploration')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{
                 'use_sim_time': True, 'autostart': True,
                 'node_names': ['controller_server', 'smoother_server', 'planner_server',
                                'behavior_server', 'bt_navigator', 'velocity_smoother'],
             }]),
    ]
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'autostart': 'true', 'use_lifecycle_manager': 'false', 'use_sim_time': 'true',
            'slam_params_file': os.path.join(package_share, 'config', 'slam_params.yaml'),
        }.items(),
    )
    exploration_node = Node(
        package='differential_drive_robot_exploration', executable='exploration_mission',
        name='exploration_mission', output='screen',
        parameters=[{
            'use_sim_time': True, 'seed': layout.seed,
            'layout_file': str(output_root / 'layout.json'),
            'world_file': str(output_root / 'navigation_arena.sdf'),
            'truth_pose_index': 6 + count,
            'output_directory': str(output_root),
            'finish_behavior': LaunchConfiguration('finish_behavior'),
            'record_results': ParameterValue(LaunchConfiguration('record_results'), value_type=bool),
            'target_coverage': ParameterValue(LaunchConfiguration('target_coverage'), value_type=float),
            'mission_timeout': ParameterValue(LaunchConfiguration('mission_timeout'), value_type=float),
            'goal_timeout': ParameterValue(LaunchConfiguration('goal_timeout'), value_type=float),
            'prefetch_distance': ParameterValue(LaunchConfiguration('prefetch_distance'), value_type=float),
            'handoff_distance': ParameterValue(LaunchConfiguration('handoff_distance'), value_type=float),
            'goal_update_interval': ParameterValue(LaunchConfiguration('goal_update_interval'), value_type=float),
            'frontier_min_cells': ParameterValue(LaunchConfiguration('frontier_min_cells'), value_type=int),
            'exhaustion_cycles': ParameterValue(LaunchConfiguration('exhaustion_cycles'), value_type=int),
            'max_recovery_spins': ParameterValue(
                LaunchConfiguration('max_recovery_spins'), value_type=int
            ),
            'stop_on_incomplete': ParameterValue(
                LaunchConfiguration('stop_on_incomplete'), value_type=bool
            ),
        }],
    )
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_roots),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             output='screen', parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='exploration_ros_gz_bridge',
             output='screen', parameters=[{'config_file': os.path.join(package_share, 'config', 'ground_truth_bridge.yaml'),
                                           'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', name='spawn_differential_drive_robot', output='screen',
             arguments=['-world', 'navigation_arena', '-file', urdf_file, '-name', 'differential_drive_robot',
                        '-x', '-5.0', '-y', '0.0', '-z', '0.08', '-Y', '0.0']),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[os.path.join(navigation_share, 'config', 'ekf.yaml'), {'use_sim_time': True}],
             remappings=[('odometry/filtered', '/odometry/filtered')]),
        slam_launch,
        *nav_nodes,
        Node(package='rviz2', executable='rviz2', name='exploration_rviz', output='screen',
             arguments=['-d', os.path.join(package_share, 'config', 'exploration.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
        exploration_node,
        RegisterEventHandler(OnProcessExit(
            target_action=exploration_node,
            on_exit=_shutdown_once,
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('differential_drive_robot_simulation'), 'launch', 'session.launch.py'))),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('obstacle_count', default_value='6'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('record_results', default_value='true'),
        DeclareLaunchArgument('finish_behavior', default_value='hold', choices=['hold', 'shutdown'],
                              description='Keep the stopped robot and final results visible, or close the stack.'),
        DeclareLaunchArgument(
            'target_coverage', default_value='98.0',
            description=(
                'Minimum reachable-free-space coverage required for successful completion. '
                'Reaching this value alone never stops exploration.'
            ),
        ),
        DeclareLaunchArgument(
            'mission_timeout', default_value='0.0',
            description='Maximum simulated seconds; 0 keeps exploring until completeness.',
        ),
        DeclareLaunchArgument('goal_timeout', default_value='45.0'),
        DeclareLaunchArgument('prefetch_distance', default_value='2.0',
                              description='Start preparing the next frontier this far along the path before arrival.'),
        DeclareLaunchArgument('handoff_distance', default_value='1.0',
                              description='Replace the goal during travel when remaining path distance falls below this.'),
        DeclareLaunchArgument('goal_update_interval', default_value='2.0',
                              description='Minimum simulated seconds between rolling target updates.'),
        DeclareLaunchArgument('frontier_min_cells', default_value='5'),
        DeclareLaunchArgument('exhaustion_cycles', default_value='3'),
        DeclareLaunchArgument(
            'max_recovery_spins', default_value='3',
            description='Maximum below-target recovery/replanning passes; at most one spin per location.',
        ),
        DeclareLaunchArgument(
            'stop_on_incomplete', default_value='true',
            description=(
                'When false, keep retrying indefinitely if frontier exhaustion occurs below '
                'the target coverage.'
            ),
        ),
        DeclareLaunchArgument('generated_world_directory', default_value='/tmp/differential_drive_exploration/worlds'),
        DeclareLaunchArgument('output_directory', default_value=str(Path.home() / 'ROS_Maps' / 'exploration')),
        OpaqueFunction(function=_launch_setup),
    ])
