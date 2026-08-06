from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
import os

def launch_setup(context, *args, **kwargs):
    # Get the map_name from launch configuration
    map_name = LaunchConfiguration('map_name').perform(context)
    
    # Set the map directory to the source workspace location
    home_dir = os.path.expanduser('~')
    maps_dir = os.path.join(home_dir, 'bumpy_ws', 'src', 'bumpy_slam', 'maps')
    
    # If map_name is still the default 'prompt', ask the user
    if map_name == 'prompt':
        print("\n" + "="*50)
        map_name = input("Enter the name for your map: ").strip()
        if not map_name:
            map_name = 'my_map'
            print(f"No name provided, using default: {map_name}")
        print("="*50 + "\n")
    
    # Make sure the directory exists
    os.makedirs(maps_dir, exist_ok=True)
    
    full_map_path = os.path.join(maps_dir, map_name)
    
    # Map saver node
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65
        }],
        arguments=[
            '-f', full_map_path
        ]
    )
    
    # Output message
    print_cmd = ExecuteProcess(
        cmd=['bash', '-c', f'echo "Map will be saved to: {full_map_path}.pgm and {full_map_path}.yaml"'],
        output='screen'
    )
    
    return [print_cmd, map_saver_node]

def generate_launch_description():
    # Declare launch arguments
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='prompt',
        description='Name of the map to save (use "prompt" to be asked interactively)'
    )
    
    # Create and return the launch description
    ld = LaunchDescription()
    ld.add_action(declare_map_name_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
