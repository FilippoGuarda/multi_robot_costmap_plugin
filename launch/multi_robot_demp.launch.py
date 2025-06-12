#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('multi_robot_costmap_plugin')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespaces = LaunchConfiguration('robot_namespaces')
    nav_params_file = LaunchConfiguration('nav_params_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_robot_namespaces_cmd = DeclareLaunchArgument(
        'robot_namespaces',
        default_value="['robot1', 'robot2', 'robot3']",
        description='List of robot namespaces'
    )
    
    declare_nav_params_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for navigation'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add the declared arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_namespaces_cmd)
    ld.add_action(declare_nav_params_cmd)

    # Add example nodes (you would typically launch your full nav2 stack here)
    example_node = Node(
        package='multi_robot_costmap_plugin',
        executable='multi_robot_demo_node',
        name='multi_robot_demo',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_namespaces': robot_namespaces}
        ],
        output='screen'
    )
    
    # ld.add_action(example_node)

    return ld