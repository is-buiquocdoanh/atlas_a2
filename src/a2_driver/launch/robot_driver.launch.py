from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="a2_driver",
            executable="driver_node.py",
            output="screen",
            parameters=[{"port": "/dev/ttyUSB0"}]
        )

    ])