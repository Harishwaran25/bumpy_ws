from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bumpy7',
        description='Namespace for the robot'
    )

    namespace = LaunchConfiguration('namespace')

    description_pkg_share = get_package_share_directory('bumpy_description')

    xacro_file = os.path.join(description_pkg_share, 'urdf', 'bumpy.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'namespace': 'bumpy7'}
    )
    robot_urdf = robot_description_config.toxml()

    # ── robot_state_publisher ─────────────────────────────────────────────────
    # FIX: remap /bumpy7/tf → /tf and /bumpy7/tf_static → /tf_static
    # Without this, static transforms (base_link, lidar_link, etc.) are published
    # to /bumpy7/tf_static which nobody listens to → 22s stale transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': robot_urdf}],
        remappings=[
            ('/tf',        '/tf'),
            ('/tf_static', '/tf_static'),
        ],
        output='screen'
    )

    # ── YLidar X2 ─────────────────────────────────────────────────────────────
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'port':                  '/dev/ttyUSB0',
            'frame_id':              'bumpy7/lidar_link',
            'ignore_array':          '',
            'baudrate':              115200,
            'lidar_type':            1,
            'device_type':           0,
            'sample_rate':           3,
            'abnormal_check_count':  4,
            'fixed_resolution':      True,
            'reversion':             True,
            'inverted':              True,
            'auto_reconnect':        True,
            'isSingleChannel':       True,
            'intensity':             False,
            'support_motor_dtr':     True,
            'angle_max':             180.0,
            'angle_min':            -180.0,
            'range_max':             12.0,
            'range_min':              0.1,
            'frequency':             10.0,
            'invalid_range_is_inf':  False,
        }]
    )

    # ── Static transform publisher for LiDAR ──────────────────────────────────
    # FIX: same remapping needed - static_transform_publisher is also namespaced
    lidar_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_transform_publisher',
        namespace=namespace,
        arguments=[
            '0.16', '0', '0.05', '0', '0', '0',
            'bumpy7/base_footprint', 'bumpy7/lidar_link'
        ],
        remappings=[
            ('/tf',        '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )

    # ── Joint state publisher ─────────────────────────────────────────────────
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': False}]
    )

    # ── EKF localization ──────────────────────────────────────────────────────
    # FIX: EKF publishes odom->base_footprint TF - also needs global /tf
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('bumpy_description'),
            'config', 'ekf.yaml'
        )],
        remappings=[
            ('/tf',        '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )
     return LaunchDescription([
        namespace_arg,
        robot_state_publisher_node,
        lidar_node,
        lidar_transform_node,
        joint_state_publisher_node,
        robot_localization,
    ])
