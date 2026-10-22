import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package locations
    dojo_share = get_package_share_directory('dojo_robot')
    four_wheels_share = get_package_share_directory('four_wheels_robot')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    # Dojo gamefield world
    world_path = os.path.join(
        dojo_share,
        'worlds',
        'gamefield.world'
    )

    models_path = os.path.join(
        dojo_share,
        'models'
    )

    # Four-wheel robot description
    four_wheels_xacro = os.path.join(
        four_wheels_share,
        'urdf',
        'model.urdf.xacro'
    )

    four_wheels_description = ParameterValue(
        Command(['xacro ', four_wheels_xacro]),
        value_type=str
    )

    # Make Gazebo find the Dojo gamefield models
    set_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # Start Gazebo with the Dojo gamefield
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items()
    )

    # Bridge Gazebo topics to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                dojo_share,
                'config',
                'bridge.yaml'
            )
        }],
        output='screen'
    )

    # Publish four-wheel robot TF
    four_wheels_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': four_wheels_description,
            'frame_prefix': 'four_wheels_robot/'
        }],
        output='screen'
    )

    # Spawn only the four-wheel robot
    spawn_four_wheels = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            '/robot_description',
            '-name',
            'four_wheels_robot',
            '-x',
            '2.0',
            '-y',
            '0',
            '-z',
            '0.30'
        ],
        output='screen'
    )
    rviz_config = os.path.join(
        dojo_share,
        'rviz',
        'dojo_mapping.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        bridge,
        four_wheels_state_publisher,
        spawn_four_wheels,
        rviz
    ])