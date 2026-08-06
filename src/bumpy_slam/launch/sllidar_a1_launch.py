#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    prefix_address = get_package_share_directory('bumpy_slam') 
    config_directory = os.path.join(prefix_address, 'config')
    slam_config = 'slam.lua'
    
    # Cartographer parameters
    res = LaunchConfiguration('resolution', default='0.05')
    publish_period = LaunchConfiguration('publish_period_sec', default='1.0')
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration = LaunchConfiguration('exploration')
    
    # RPLidar A1 parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # SLAM parameters
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='False',
            description='Flag to enable use_sim_time'
        ),
        
        DeclareLaunchArgument(
            name='exploration', 
            default_value='True',
            description='Flag to enable exploration mode'
        ),
        
        DeclareLaunchArgument(
            'resolution',
            default_value=res,
            description='Configure the resolution'
        ),
        
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period,
            description='Publish period in seconds'
        ),
        
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=config_directory,
            description='Path to the .lua files'
        ),
        
        DeclareLaunchArgument(
            'slam_configuration_basename',
            default_value=slam_config,
            description='Name of .lua file to be used for SLAM'
        ),
        
        DeclareLaunchArgument(
            'localization_configuration_basename',
            default_value=slam_config,
            description='Name of .lua file to be used for localization'
        ),
        
        # RPLidar A1 parameters
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'
        ),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'
        ),
        
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'
        ),
        
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'
        ),
        
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),
        
        # RPLidar A1 Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port, 
                'serial_baudrate': serial_baudrate, 
                'frame_id': frame_id,
                'inverted': inverted, 
                'angle_compensate': angle_compensate
            }],
            output='screen'
        ),
        
        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='as21_cartographer_node',
            arguments=[
                '-configuration_directory', config_directory,
                '-configuration_basename', slam_config
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/scan', '/scan'),
            ],
            output='screen'
        ),
        
        # Occupancy Grid Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            arguments=[
                '-resolution', res,
                '-publish_period_sec', publish_period
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
