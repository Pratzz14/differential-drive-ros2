#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory(
        'differential_drive_robot_simulation'
    )
    description_share = get_package_share_directory(
        'differential_drive_robot_description'
    )

    world_file = os.path.join(simulation_share, 'worlds', 'test_world.sdf')
    bridge_file = os.path.join(simulation_share, 'config', 'bridge.yaml')
    rviz_file = os.path.join(simulation_share, 'config', 'simulation.rviz')
    urdf_file = os.path.join(
        description_share, 'urdf', 'differential_drive_robot.urdf'
    )

    with open(urdf_file, 'r', encoding='utf-8') as urdf_stream:
        robot_description = urdf_stream.read()

    # Gazebo resolves package:// resources relative to resource roots. Include
    # both the install share directory and its parent so this works with the
    # standard ament install layout and with direct package overlays.
    resource_roots = os.pathsep.join(
        [os.path.dirname(description_share), description_share]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', value=resource_roots
        ),
        gazebo_launch,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'robot_description': robot_description,
                }
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='bridge_node',
            name='ros_gz_bridge',
            output='screen',
            parameters=[
                {'config_file': bridge_file, 'use_sim_time': True}
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_differential_drive_robot',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-file',
                urdf_file,
                '-name',
                'differential_drive_robot',
                '-z',
                '0.08',
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}],
        ),
    ])
