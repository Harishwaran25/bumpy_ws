#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Fixed namespace for this robot
    namespace = 'bumpy7'

    # Package paths
    bumpy_description_pkg = FindPackageShare('bumpy_description')
    bumpy_firmware_pkg = FindPackageShare('bumpy_firmware')

    # Robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bumpy_description_pkg,
                'launch',
                'robot.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace
        }.items()
    )

    # Motor control launch
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bumpy_firmware_pkg,
                'launch',
                'motor_control.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace
        }.items()
    )

    # ==========================
    # SENSOR + STATUS NODES
    # ==========================

    camera_node = Node(
        package='bumpy_sensors',
        executable='camera_node',
        name='camera_node',
        output='screen',
        remappings=[
            ('image_raw', '/bumpy7/image_raw')
        ]
    )

    imu_node = Node(
        package='bumpy_sensors',
        executable='imu_node',
        name='imu_node',
        output='screen'
    )

    web_server_node = Node(
        package='bumpy_sensors',
        executable='web_server_node',
        name='web_server_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Robot Status Node (namespaced)
    robot_status_node = Node(
        package='bumpy_sensors',
        executable='robot_status_node',
        name='robot_status_node',
        output='screen'
    )

    # Group all robot-specific nodes under namespace
    sensors_group = GroupAction([
        PushRosNamespace(namespace),
        camera_node,
        imu_node,
        web_server_node,
        robot_status_node
    ])

    

    # ==========================
    # Launch Description
    # ==========================

    ld = LaunchDescription()

    ld.add_action(robot_description_launch)
    ld.add_action(motor_control_launch)
    ld.add_action(sensors_group)

    
    return ld
