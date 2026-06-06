from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_camera',
        parameters=[
            {"video_device": "/dev/video2"},
            {"image_size": [352, 288]},
            {"pixel_format": "YUYV"},
            {"frame_rate": 30},
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

    return LaunchDescription([
        camera_node,
        republish_node,
    ])
