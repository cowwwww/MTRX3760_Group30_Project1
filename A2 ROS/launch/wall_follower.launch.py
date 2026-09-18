"""ROS 2 controller for an already running robot or simulation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mtrx3760_project1')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('stamped_cmd_vel', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'wall_follower_ros2.yaml')),
        Node(package='mtrx3760_project1', executable='turtlebot3_drive',
             name='project1_wall_follower', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'stamped_cmd_vel': ParameterValue(LaunchConfiguration('stamped_cmd_vel'), value_type=bool),
             }],
             remappings=[('scan', LaunchConfiguration('scan_topic')),
                         ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                         ('odom', LaunchConfiguration('odom_topic'))]),
    ])
