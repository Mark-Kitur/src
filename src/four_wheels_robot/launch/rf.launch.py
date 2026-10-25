from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',          # Your LiDAR topic
                'odom_topic': '/odom_rf2o',                # Published Odometry topic
                'publish_tf': False,                   # Publishes odom -> base_link
                'base_frame_id': 'base_footprint',         # Robot base frame
                'odom_frame_id': 'odom',              # Odometry frame
                'init_pose_from_topic': '',
                'freq': 20.0                          # Match or exceed your LiDAR's Hz
            }],
        ),
    ])
