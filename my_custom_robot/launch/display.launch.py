#!/usr/bin/env python3
"""Launch file to visualize the my_custom_robot URDF in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('my_custom_robot')
    urdf_path = os.path.join(package_share, 'urdf', 'my_robot.urdf')

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f'URDF file not found: {urdf_path}')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description_content = urdf_file.read()

    rviz_config_path = os.path.join(package_share, 'rviz', 'robot_display.rviz')
    rviz_arguments = []
    if os.path.exists(rviz_config_path):
        rviz_arguments = ['-d', rviz_config_path]

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_arguments,
    )

    return LaunchDescription([
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz_node,
    ])
