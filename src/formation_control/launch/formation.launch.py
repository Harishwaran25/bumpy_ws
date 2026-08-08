from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('formation_control')
    

    # Create the formation controller node
    formation_controller_node = Node(
        package='formation_control',
        executable='3robot',
        name='formation_controller',
        output='screen',
    )

   # Create the formation controller node
    formation_controller_node_2 = Node(
        package='formation_control',
        executable='3robot2',
        name='formation_controller2',
        output='screen',
    )
    
    # Return the launch description
    return LaunchDescription([
        formation_controller_node,
        formation_controller_node_2
    ])
