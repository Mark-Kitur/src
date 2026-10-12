import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",default_value="true", description="Use sim time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    pkg_name = "four_wheels_robot"
    urdf_file = 'model.urdf.xacro'


    pkg_share_dir = get_package_share_directory(pkg_name)

    urdf_path = os.path.join(pkg_share_dir, "urdf", urdf_file)
    robot_description = xacro.process_file(urdf_path).toxml()

    bridge_conf = os.path.join(pkg_share_dir,'config', "bridge.yaml")

    world_path = os.path.join(pkg_share_dir,"world", 'simple_.sdf')

    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        name='rviz2',
        parameters=[{"use_sim_time":use_sim_time}]
    )

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            "gz_args": [world_path, " -r"], 
            'use_sim_time': use_sim_time
        }.items(),
    )

    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}] 
    )
    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'four_wheels_robot', 
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '-1.0',
            '-z', '0.6'
        ],
        output='screen'
    )

    # Load controllers via spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['velocity_controller'],
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    delayed_spawners = TimerAction(
        period=3.0,  
        actions=[
            joint_state_broadcaster_spawner,
            velocity_controller_spawner,imu_broadcaster_spawner
        ]
    )


    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name="parameter_bridge",
        parameters=[
            {'config_file': bridge_conf,
             'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share_dir, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}])
    
    # static_trans=Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     arguments=['0', '0', '0', '0', '0', '0','base_footprint', 'four_wheels_robot/base_footprint/lidar_link']
    # )
    ld = LaunchDescription([
        declare_use_sim_time,
        #gz_sim,
        #clock_bridge,
        robot_state_publisher,
        # joint_state_publisher_gui,
        rviz,
        spawn_entity,
        delayed_spawners,
        robot_localization_node,
    ])

    return ld
