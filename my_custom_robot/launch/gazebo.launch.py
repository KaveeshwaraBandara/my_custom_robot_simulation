#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_custom_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f'URDF file not found: {urdf_path}')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    pkg_share_parent = os.path.dirname(pkg_share)

    resource_paths = [pkg_share_parent, pkg_share]
    existing_resource = os.environ.get('GZ_SIM_RESOURCE_PATH')
    if existing_resource:
        resource_paths.append(existing_resource)

    model_paths = [pkg_share_parent, pkg_share]
    existing_models = os.environ.get('GAZEBO_MODEL_PATH')
    if existing_models:
        model_paths.append(existing_models)

    # Launch an empty Gazebo (gz) world
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # Run headless + verbose on a minimal world with sensor support
        launch_arguments={'gz_args': '-r -v 3 worlds/empty_sensor.sdf'}.items()
    )

    # Publish TF from your URDF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
        output='screen'
    )

    # Spawn the URDF into Gazebo (ros_gz_sim “create” tool)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_custom_robot', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        parameters=[
            {
                'config_file': bridge_config,
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_paths)),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_paths)),
        gz_launch,
        rsp,
        spawn,
        bridge,
    ])
