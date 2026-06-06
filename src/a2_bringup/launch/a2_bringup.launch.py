import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # RPLidar
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch', 'rplidar_a2m12_launch.py'
            )
        )
    )

    # TF tĩnh: base_link → laser
    # Chỉnh x y z và quaternion (qx qy qz qw) theo vị trí lắp lidar thực tế
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=[
            '0', '0', '0',      # x y z (m)
            '0', '0', '1', '0', # qx qy qz qw  (180° quanh Z)
            'base_link',
            'laser',
        ],
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

    return LaunchDescription([
        rplidar_launch,
        tf_base_laser,
        scan_relay,
        rf2o_node,
    ])
