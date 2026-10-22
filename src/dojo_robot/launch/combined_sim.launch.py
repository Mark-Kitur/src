import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package locations
    dojo_share = get_package_share_directory('dojo_robot')
    four_wheels_share = get_package_share_directory('four_wheels_robot')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    # Dojo game field
    world_path = os.path.join(
        dojo_share, 'worlds', 'gamefield.world'
    )
    models_path = os.path.join(dojo_share, 'models')

    # Robot descriptions
    dojo_xacro = os.path.join(
        dojo_share, 'urdf', 'dojo_robot.xacro'
    )
    four_wheels_xacro = os.path.join(
        four_wheels_share, 'urdf', 'model.urdf.xacro'
    )

    # Make Gazebo find the game-field model
    set_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # Start Gazebo only once
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
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

    # Generate the Dojo robot description
    dojo_description = ParameterValue(
        Command(['xacro ', dojo_xacro]),
        value_type=str
    )

    # Generate the four-wheel robot description
    four_wheels_description = ParameterValue(
        Command(['xacro ', four_wheels_xacro]),
        value_type=str
    )

    # Publish Dojo robot TF
    dojo_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='dojo_robot',
        parameters=[{
            'robot_description': dojo_description,
            'frame_prefix': 'dojo_robot/'
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

    # Spawn the Dojo robot
    spawn_dojo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/dojo_robot/robot_description',
            '-name', 'dojo_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # Spawn the four-wheel robot away from the Dojo robot
    spawn_four_wheels = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-topic', '/robot_description',
        '-name', 'four_wheels_robot',
        '-x', '2.0',
        '-y', '0',
        '-z', '0.30'
    ],
    output='screen'
)
    return LaunchDescription([
        set_model_path,
        gazebo,
        bridge,
        dojo_state_publisher,
        four_wheels_state_publisher,
        spawn_dojo,
        spawn_four_wheels
    ])