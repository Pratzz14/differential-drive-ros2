#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from differential_drive_robot_navigation.obstacle_generator import generate_world


ROBOT_START_X = -5.0
ROBOT_START_Y = 0.0
ROBOT_START_Z = 0.08
ROBOT_START_YAW = 0.0


def _enabled(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context):
    navigation_share = get_package_share_directory('differential_drive_robot_navigation')
    simulation_share = get_package_share_directory('differential_drive_robot_simulation')
    description_share = get_package_share_directory('differential_drive_robot_description')

    seed_text = LaunchConfiguration('seed').perform(context)
    requested_seed = None if seed_text.strip() in ('', '-1', 'random') else int(seed_text)
    obstacle_count = int(LaunchConfiguration('obstacle_count').perform(context))
    automatic_goal = (
        float(LaunchConfiguration('goal_x').perform(context)),
        float(LaunchConfiguration('goal_y').perform(context)),
    )
    generated_root = LaunchConfiguration('generated_world_directory').perform(context)
    layout, world_file, layout_file = generate_world(
        Path(simulation_share) / 'worlds' / 'navigation_arena.sdf.in',
        generated_root,
        requested_seed,
        obstacle_count,
        start=(ROBOT_START_X, ROBOT_START_Y),
        goal=automatic_goal,
    )

    urdf_file = os.path.join(
        description_share, 'urdf', 'differential_drive_robot.urdf'
    )
    with open(urdf_file, 'r', encoding='utf-8') as urdf_stream:
        robot_description = urdf_stream.read()

    resource_roots = os.pathsep.join(
        [os.path.dirname(description_share), description_share]
    )
    os.environ['GZ_SIM_RESOURCE_PATH'] = resource_roots

    headless = _enabled(LaunchConfiguration('headless').perform(context))
    # Launch directly, without an intermediate shell that can exit while its
    # Gazebo child survives. The launch service owns the actual simulator.
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', *(['-s'] if headless else []), str(world_file)],
        output='screen',
        on_exit=Shutdown(reason='Gazebo exited'),
    )

    nav2_parameters = os.path.join(navigation_share, 'config', 'nav2.yaml')
    ekf_parameters = os.path.join(navigation_share, 'config', 'ekf.yaml')
    bridge_parameters = os.path.join(
        navigation_share, 'config', 'navigation_bridge.yaml'
    )
    map_file = os.path.join(navigation_share, 'maps', 'navigation_map.yaml')
    rviz_file = os.path.join(navigation_share, 'config', 'navigation.rviz')

    common_nav2_parameters = [nav2_parameters, {'use_sim_time': True}]
    navigation_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=common_nav2_parameters,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=common_nav2_parameters,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=common_nav2_parameters,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=common_nav2_parameters,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=common_nav2_parameters,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=common_nav2_parameters,
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # mission_runner starts navigation after scan-confirmed localization.
                'autostart': False,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother',
                ],
            }],
        ),
    ]

    return [
        gazebo,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='navigation_ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_parameters, 'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_differential_drive_robot',
            output='screen',
            arguments=[
                '-world', 'navigation_arena',
                '-file', urdf_file,
                '-name', 'differential_drive_robot',
                '-x', str(ROBOT_START_X),
                '-y', str(ROBOT_START_Y),
                '-z', str(ROBOT_START_Z),
                '-Y', str(ROBOT_START_YAW),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_parameters, {'use_sim_time': True}],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_parameters, {'yaml_filename': map_file, 'use_sim_time': True}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                nav2_parameters,
                {
                    'use_sim_time': True,
                    'initial_pose.x': ROBOT_START_X,
                    'initial_pose.y': ROBOT_START_Y,
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': ROBOT_START_YAW,
                },
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
        *navigation_nodes,
        Node(
            package='rviz2',
            executable='rviz2',
            name='navigation_rviz',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='differential_drive_robot_navigation',
            executable='mission_runner',
            name='navigation_mission',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'seed': layout.seed,
                'layout_file': str(layout_file),
                'auto_goal': ParameterValue(
                    LaunchConfiguration('auto_goal'), value_type=bool
                ),
                'start_x': ROBOT_START_X,
                'start_y': ROBOT_START_Y,
                'start_yaw': ROBOT_START_YAW,
                'goal_x': ParameterValue(
                    LaunchConfiguration('goal_x'), value_type=float
                ),
                'goal_y': ParameterValue(
                    LaunchConfiguration('goal_y'), value_type=float
                ),
                'goal_yaw': ParameterValue(
                    LaunchConfiguration('goal_yaw'), value_type=float
                ),
                'results_directory': LaunchConfiguration('results_directory'),
                'record_results': ParameterValue(
                    LaunchConfiguration('record_results'), value_type=bool
                ),
                'mission_timeout': ParameterValue(
                    LaunchConfiguration('mission_timeout'), value_type=float
                ),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('differential_drive_robot_simulation'),
                         'launch', 'session.launch.py')
        )),
        DeclareLaunchArgument(
            'seed',
            default_value='-1',
            description='Integer obstacle seed; -1 chooses and prints a random seed.',
        ),
        DeclareLaunchArgument(
            'obstacle_count',
            default_value='6',
            description='Number of randomized stationary obstacles (0-12).',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'auto_goal',
            default_value='false',
            description=(
                'Automatically send a navigation goal. When false, use RViz '
                '2D Goal Pose to choose the destination.'
            ),
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='5.0',
            description='Automatic goal X coordinate in the map frame (metres).',
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.0',
            description='Automatic goal Y coordinate in the map frame (metres).',
        ),
        DeclareLaunchArgument(
            'goal_yaw',
            default_value='0.0',
            description='Automatic goal heading in radians.',
        ),
        DeclareLaunchArgument('mission_timeout', default_value='180.0'),
        DeclareLaunchArgument('record_results', default_value='true'),
        DeclareLaunchArgument(
            'generated_world_directory',
            default_value='/tmp/differential_drive_navigation/worlds',
        ),
        DeclareLaunchArgument(
            'results_directory',
            default_value='/tmp/differential_drive_navigation/results',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
