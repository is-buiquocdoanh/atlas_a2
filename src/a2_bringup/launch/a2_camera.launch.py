import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_camera',
        parameters=[
            {"video_device": "/dev/video0"},
            {"image_size": [640, 480]},
            {"pixel_format": "YUYV"},
            {"frame_rate": 15},
        ]
    )

    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_compressed',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/image_raw/compressed'),
            ('out', '/yolo_image_raw'),
        ]
    )
    
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 6060,          # Cổng để truy cập trên trình duyệt
            'address': '0.0.0.0',  # Cho phép tất cả các IP trong mạng truy cập
            'type': 'ros_compressed' # Ưu tiên dùng chuẩn nén để mượt hơn
        }],
        output='screen'
    )
    
    yolov8 = Node(
        package='a2_bringup',
        executable='yolov8_ros2_pt.py',
        name='yolov8_node',
        output='screen',
    )
    return LaunchDescription([
        camera_node,
        republish_node,
        web_video_server_node,
        yolov8,
    ])
