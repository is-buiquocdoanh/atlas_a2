import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

USE_SIM_TIME = False


def generate_launch_description():
    package_dir = get_package_share_directory('a2_slam')
    workspace_dir = os.path.abspath(os.path.join(package_dir, '..', '..', '..', '..'))
    maps_dir = os.path.join(workspace_dir, 'src', 'a2_maps')
    params_file = os.path.join(package_dir, 'config', 'a2_localization.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'nav2_default_view.rviz')

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(maps_dir, 'map2', 'map2.yaml'),
        description='Full path to the YAML map file to load'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map')},
            {'use_sim_time': USE_SIM_TIME}
        ]
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[params_file, {'use_sim_time': USE_SIM_TIME}]
    )

    start_localization_slam_toolbox_node = Node(
        parameters=[params_file, {'use_sim_time': USE_SIM_TIME}],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        remappings=[('/map', '/map')],
        output='screen'
    )

    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': USE_SIM_TIME,
            'autostart': True,
            'node_names': ['map_server', 'map_saver']
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        map_yaml_arg,
        map_server,
        map_saver,
        start_localization_slam_toolbox_node,
        lifecycle_manager_loc,
        rviz_node
    ])
