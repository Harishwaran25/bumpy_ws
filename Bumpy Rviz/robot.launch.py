from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Declare namespace argument with bumpy6 as default
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bumpy7',
        description='Namespace for the robot'
    )
    
    # Get namespace as a LaunchConfiguration
    namespace = LaunchConfiguration('namespace')
    
    # Package directories
    description_pkg_share = get_package_share_directory('bumpy_description')
    
    # URDF file path with namespace parameter passed to xacro
    xacro_file = os.path.join(description_pkg_share, 'urdf', 'bumpy.xacro')
    
    # Process xacro file with namespace parameter
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'namespace': 'bumpy7'}  # Hard-code bumpy6 namespace
    )
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    # RPLidar C1 node (replaced YDLidar)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # C1 uses 256000 baud rate
            'frame_id': 'bumpy7/lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Express',
        }]
    )
    
    # Static transform publisher for LiDAR
    # Using hardcoded namespaced frames
    lidar_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_transform_publisher',
        namespace=namespace,
        arguments=['0.16', '0', '0.05', '0', '0', '0', 'bumpy7/base_footprint', 'bumpy7/lidar_link']
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': False}]
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=namespace,
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bumpy_description"), "config", "ekf.yaml")],
    )

    return LaunchDescription([
        namespace_arg,
        robot_state_publisher_node,
        lidar_node,
        lidar_transform_node,
        joint_state_publisher_node,
        #robot_localization
    ])

