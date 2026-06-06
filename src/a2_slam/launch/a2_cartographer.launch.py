import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

USE_SIM_TIME = False


def generate_launch_description():
    cartographer_pkg = get_package_share_directory('a2_slam')
    rviz_config = os.path.join(cartographer_pkg, 'rviz', 'cartographer_mapping.rviz')
    configuration_directory = os.path.join(cartographer_pkg, 'config')
    configuration_basename = 'lds_2d.lua'

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename
            ],
            remappings=[('/scan', '/atlas/scan_filtered')],
            parameters=[{'use_sim_time': USE_SIM_TIME}]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': USE_SIM_TIME,
                'resolution': 0.05,
                'publish_period_sec': 1.0
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': USE_SIM_TIME}]
        )
    ])
