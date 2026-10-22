import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1. Declare the launch argument and setup runtime tracking
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use sim time"
    )
    use_sim_time_var = LaunchConfiguration('use_sim_time')
    
    pkg_name = "four_wheels_robot"
    pkg_share_dir = get_package_share_directory(pkg_name)

    # 2. Get exact file paths
    amcl_yaml = os.path.join(pkg_share_dir, 'config', 'amcl.yaml')
    nav_params = os.path.join(pkg_share_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share_dir, 'map', 'map_room.yaml')  
    
    # Common parameter dict to merge with YAML files
    common_params = {'use_sim_time': use_sim_time_var}

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        # Pass the map path using the required 'yaml_filename' key
        parameters=[{"yaml_filename": map_file}, common_params], 
    )


    # 3. Individual Nav2 Node Configurations
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_yaml, common_params],  # Pass the file directly
        arguments=['--ros-args', '--log-level', 'debug']
    )

    nav2_controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",    
        parameters=[nav_params, common_params],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    nav2_smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[nav_params, common_params]
    )
    
    nav2_planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[nav_params, common_params]
    )
    
    nav2_behaviour = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[nav_params, common_params]
    )
    
    nav2_navigator = Node(
        package="nav2_bt_navigator",  # Fixed package name (was nav2_navigator)
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[nav_params, common_params]
    )
    
    nav2_waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[nav_params, common_params]
    )
    
    nav2_collision_avoidance = Node(
        package="nav2_collision_monitor",  # Fixed package name (was nav2_collision_avoidance)
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[nav_params, common_params]
    )
    
    # 4. Lifecycle Manager controls system startup transition
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time_var,
            'autostart': True,
            'node_names': [
                'map_server',        # 1. Map server must come first
                'amcl',              # 2. AMCL localization must come second
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'collision_monitor'
            ]
        }]
    )

    static_transform_publisher_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time_var}]
    )   
    return LaunchDescription([
        declare_use_sim_time,
        amcl_node,
        nav2_controller,
        nav2_smoother,
        nav2_planner,
        nav2_behaviour,
        nav2_navigator,
        nav2_waypoint_follower,
        nav2_collision_avoidance,
        map_server_node,
        #static_transform_publisher_map_to_odom,
        lifecycle_manager
    ])
