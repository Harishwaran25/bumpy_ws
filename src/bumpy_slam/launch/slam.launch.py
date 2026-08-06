from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bumpy7',
        description='Namespace for the robot'
    )
    
    # Get namespace as a LaunchConfiguration
    namespace = LaunchConfiguration('namespace')
    
    # Get package share directory
    pkg_share = FindPackageShare('bumpy_slam')
    
    # Launch arguments for SLAM configuration
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml']),
        description='SLAM parameters file'
    )
    
    # SLAM Toolbox node with namespace and topic remapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,  # Added namespace
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Frame IDs with namespace
                'odom_frame': 'bumpy7/odom',
                'map_frame': 'map',
                'base_frame': 'bumpy7/base_footprint',
                'scan_topic': '/bumpy7/scan',  # Explicit scan topic
            }
        ],
        remappings=[
            ('/scan', '/bumpy7/scan'),  # Remap scan topic
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        params_file_arg,
        slam_toolbox_node
    ])
