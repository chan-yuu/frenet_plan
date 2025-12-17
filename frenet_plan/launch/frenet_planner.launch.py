from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('frenet_plan')
    config_file = os.path.join(pkg_dir, 'config', 'frenet_params.yaml')
    
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
    
    return LaunchDescription([
        frenet_planner_node
    ])
