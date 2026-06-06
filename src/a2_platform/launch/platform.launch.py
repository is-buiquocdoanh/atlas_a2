import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('a2_platform'),
        'config', 'ports.yaml'
    )

    return LaunchDescription([

        Node(
            package='a2_platform',
            executable='battery_node.py',
            name='battery_node',
            output='screen',
            parameters=[params],
        ),

        Node(
            package='a2_platform',
            executable='collision_detect_node.py',
            name='collision_detect',
            output='screen',
            parameters=[params],
        ),

        Node(
            package='a2_platform',
            executable='mag_sensor_node.py',
            name='mag_sensor_node',
            output='screen',
            parameters=[params],
        ),

    ])
