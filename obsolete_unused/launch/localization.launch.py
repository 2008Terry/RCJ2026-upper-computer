import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to map file
    map_file_path = os.path.join(
        get_package_share_directory('rcj_localization'),
        'maps',
        'rcj_map.yaml'
    )

    return LaunchDescription([
        # Start Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_file_path}]  # Map YAML file path
        ),

        # Start Lifecycle Manager to activate Map Server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}]  # Autostart and manage the map server lifecycle
        ),

        # Start Localization Node
        Node(
            package='rcj_localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[{'num_particles': 1000}]  # Particle filter particle count
        ),
    ])
