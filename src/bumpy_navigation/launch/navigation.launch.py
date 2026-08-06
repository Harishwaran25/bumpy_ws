import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    amr = get_package_share_directory('bumpy_navigation')

    namespace     = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam          = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    params_file   = LaunchConfiguration('params_file')
    autostart     = LaunchConfiguration('autostart')
    use_respawn   = LaunchConfiguration('use_respawn')
    log_level     = LaunchConfiguration('log_level')

    # ── Parameter file setup ─────────────────────────────────────────────────
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
    }

    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace))

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # ── Declare arguments ────────────────────────────────────────────────────
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Whether to run SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/bumpy7/bumpy_ws/src/bumpy_slam/maps/room_map.yaml',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(amr, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # ── robot_state_publisher on laptop so Nav2 has fresh TF locally ─────────
    _desc_share = get_package_share_directory('bumpy_description')
    _xacro_file = os.path.join(_desc_share, 'urdf', 'bumpy7.xacro')
    _robot_urdf = xacro.process_file(_xacro_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': _robot_urdf,
            'use_sim_time': False,
        }],
    )

    # =========================================================================
    # LOCALIZATION NODES
    # =========================================================================
    map_server_node = Node(
        condition=IfCondition(PythonExpression(['not ', slam])),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=False,
        parameters=[configured_params],
    )

    amcl_node = Node(
        condition=IfCondition(PythonExpression(['not ', slam])),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=False,
        parameters=[configured_params],
    )

    lifecycle_localization_node = Node(
        condition=IfCondition(PythonExpression(['not ', slam])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # =========================================================================
    # NAVIGATION NODES
    # controller_server publishes directly to /bumpy7/cmd_vel
    # velocity_smoother and cmd_vel_relay removed
    # =========================================================================
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=False,
        parameters=[configured_params],
        remappings=[
            ('cmd_vel',   '/bumpy7/cmd_vel'),  # direct to robot
            ('/odom',     '/bumpy7/odom'),
            ('/scan',     '/bumpy7/scan'),
        ],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=False,
        parameters=[configured_params],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=False,
        parameters=[configured_params],
        remappings=[
            ('/odom', '/bumpy7/odom'),
        ],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=False,
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', '/bumpy7/cmd_vel'),  # direct to robot
            ('/odom',   '/bumpy7/odom'),
            ('/scan',   '/bumpy7/scan'),
        ],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=False,
        parameters=[configured_params],
        remappings=[
            ('/odom', '/bumpy7/odom'),
        ],
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=False,
        parameters=[configured_params],
    )

    lifecycle_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    # ── Build LaunchDescription ───────────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Robot TF (local to laptop)
    ld.add_action(robot_state_publisher_node)

    # Localization
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_localization_node)

    # Navigation
    ld.add_action(controller_server_node)
    ld.add_action(smoother_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(waypoint_follower_node)
    ld.add_action(lifecycle_navigation_node)

    return ld
