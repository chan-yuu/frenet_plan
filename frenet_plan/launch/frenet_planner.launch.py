'''
Author: CYUN && cyun@tju.enu.cn
Date: 2025-12-18 20:46:16
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-12-18 20:53:35
FilePath: /undefined/home/cyun/Videos/frenet_plan/src/frenet_plan/launch/frenet_planner.launch.py
Description: 

Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
'''
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('frenet_plan')
    config_file = os.path.join(pkg_dir, 'config', 'frenet_params.yaml')

    default_rviz_config_path = os.path.join(pkg_dir, 'rviz', 'frenet.rviz')
    
    frenet_planner_node = Node(
        package='frenet_plan',
        executable='frenet_plan_node',
        name='frenet_planner',
        output='screen',
        parameters=[config_file] if os.path.exists(config_file) else [],
        remappings=[
            ('/local_path_plan', '/local_path_plan'),
            ('/local_vel_cmd', '/local_vel_cmd'),
            ('/odom', '/odom'),
            ('/costmap', '/costmap'),
            ('/frenet_path', '/frenet_path'),
            ('/frenet_vel', '/frenet_vel'),
            ('/frenet_trajectories', '/frenet_trajectories'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_frenet',
        # output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig', default=default_rviz_config_path)],
    )
    
    return LaunchDescription([
        frenet_planner_node,
        rviz_node
    ])
