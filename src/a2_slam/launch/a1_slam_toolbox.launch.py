import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

USE_SIM_TIME = False


def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map_file')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('a2_slam'), 'config', 'a2_slam_toolbox.yaml'),
        description='Full path to the slam_toolbox ROS2 parameters file'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to existing posegraph (no extension) for incremental mapping; '
                    'empty = start fresh SLAM'
    )

    rviz_config = os.path.join(
        get_package_share_directory('a2_slam'), 'rviz', 'slam_toolbox_default.rviz')

    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': USE_SIM_TIME,
             'map_file_name': map_file},
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(rviz_node)
    return ld
