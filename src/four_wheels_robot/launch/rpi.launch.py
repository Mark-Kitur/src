import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_name = get_package_share_directory("four_wheels_robot")
    xacro_file = "model.urdf.xacro"
    xacro_file_path = os.path.join(pkg_name,"urdf", xacro_file)

    controllers = os.path.join(pkg_name,"config","wheels_controllers.yaml")

    robot_description = xacro.process_file(xacro_file_path).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ]
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            controllers
        ]
    )
    

    # Controllers Joint broadcaster, IMU, Diff drive
    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            name="joint_state_broadcaster_spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager"
                ]
            )



    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['imu_sensor_broadcaster',"--controller-manager",'/controller_manager']
    )

    velocity_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=['velocity_controller',"--controller-manager",'/controller_manager']
    )
    robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_name, 'config', 'ekf.yaml')])

    ld = LaunchDescription()

    timer_action = TimerAction(
    period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            velocity_spawner,
            imu_broadcaster_spawner,
        ]
    )

    rf=Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',          # Your LiDAR topic
                'odom_topic': '/odom',                # Published Odometry topic
                'publish_tf': True,                   # Publishes odom -> base_link
                'base_frame_id': 'base_footprint',         # Robot base frame
                'odom_frame_id': 'odom',              # Odometry frame
                'init_pose_from_topic': '',
                'freq': 20.0                          # Match or exceed your LiDAR's Hz
            }],
        )

    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(timer_action)
    #ld.add_action(robot_localization_node)
    ld.add_action(rf)
    return ld
