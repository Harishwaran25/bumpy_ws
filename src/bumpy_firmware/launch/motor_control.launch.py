from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='bumpy7',
        description='Namespace for the robot')
    namespace = LaunchConfiguration('namespace')

    speed_multiplier_arg = DeclareLaunchArgument(
        'speed_multiplier', default_value='1.0',
        description='Overall speed multiplier (0.0-1.0)')
    rotation_multiplier_arg = DeclareLaunchArgument(
        'rotation_multiplier', default_value='0.5',
        description='Rotation speed multiplier (0.0-1.0)')
    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale', default_value='1.0',
        description='Additional scaling for linear movement (0.0-1.0)')
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel', default_value='0.25',
        description='Maximum linear velocity from cmd_vel (m/s)')
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel', default_value='1.0',
        description='Maximum angular velocity from cmd_vel (rad/s)')

    motor_control_node = Node(
        package='bumpy_firmware',
        executable='control',
        name='motor_control',
        output='screen',
        parameters=[{
            'speed_multiplier': LaunchConfiguration('speed_multiplier'),
            'rotation_multiplier': LaunchConfiguration('rotation_multiplier'),
            'linear_scale': LaunchConfiguration('linear_scale'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
        }])

    ticks_publisher_node = Node(
        package="bumpy_firmware",
        executable="tick",
        name="ticks_publisher")

    diff_node = Node(
        package="bumpy_firmware",
        executable="diff",
        name="diff",
        parameters=[{
            'publish_tf': False,
            'base_width': 0.069,
            'ticks_meter': 6193,
            'base_frame_id': 'bumpy7/base_footprint',
            'odom_frame_id': 'bumpy7/odom',
        }])

    motor_nodes_group = GroupAction([
        PushRosNamespace(namespace),
        motor_control_node,
        ticks_publisher_node,
        diff_node,
    ])

    return LaunchDescription([
        namespace_arg,
        speed_multiplier_arg,
        rotation_multiplier_arg,
        linear_scale_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        motor_nodes_group,
    ])
