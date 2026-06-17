import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Include platform launch file để khởi chạy các node liên quan đến phần cứng của robot (đọc cảm biến, v.v.)
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('a2_platform'), 'launch', 'platform.launch.py')
        )
    )

    urdf = os.path.join(get_package_share_directory('a2_bringup'), 'urdf', 'a2_robot.urdf')
    robot_description = open(urdf).read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Relay /scan → /atlas/scan_filtered
    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        parameters=[{
            'input_topic':  '/scan',
            'output_topic': '/atlas/scan_filtered',
        }],
    )

    rf2o_node = Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/atlas/scan_filtered',
                'odom_topic': '/atlas/odom', # có thể sử dụng /odom_rf2o
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 30.0
            }],
        )
    
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
    
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('a2_bringup'), 'launch', 'joystick.launch.py')
        )
    )
    
    # driver 
    driver_node = Node(
        package='a2_driver',
        executable='driver_node.py',
        name='driver_node',
        output='screen',
        parameters=[{"port": "/dev/usbcan"}]
    )
    
    # API
    api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('atlas_api'), 'launch', 'atlas_api_real.launch.py')
        )
    )
    
    return LaunchDescription([
        platform_launch,
        # robot_state_publisher,
        # scan_relay,
        # rf2o_node,
        camera_node,
        # republish_node,
        web_video_server_node,
        # yolov8,
        joy_launch,
        driver_node,
        # api_launch,
    ])
