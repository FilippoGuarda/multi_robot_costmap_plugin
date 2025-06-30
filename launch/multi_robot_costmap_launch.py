#!/usr/bin/env python3

# Copyright 2025 Filippo Guarda
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = FindPackageShare('multi_robot_costmap_plugin')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    robots_arg = DeclareLaunchArgument(
        'robots',
        default_value="['robot1', 'robot2', 'robot3']",
        description='List of robot names'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'multi_robot_costmap.yaml']),
        description='Path to config file'
    )

    # Global Costmap Fusion Node
    fusion_node = Node(
        package='multi_robot_costmap_plugin',
        executable='global_costmap_fusion_node',
        name='global_costmap_fusion',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add any topic remappings if needed
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robots_arg,
        config_file_arg,
        fusion_node
    ])