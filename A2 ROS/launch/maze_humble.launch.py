"""Gazebo Classic maze for ROS 2 Humble, with TurtleBot3 laser and camera.

Requires the matching Humble turtlebot3_gazebo package. For Jazzy's Gazebo,
start the lab's world separately and use wall_follower.launch.py.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(directory, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(directory, filename)),
        launch_arguments=arguments.items())


def generate_launch_description():
    share = get_package_share_directory('mtrx3760_project1')
    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    model = os.environ.get('TURTLEBOT3_MODEL', '')
    if model not in ('waffle', 'waffle_pi'):
        raise RuntimeError('Set TURTLEBOT3_MODEL=waffle or waffle_pi for a simulated camera')
    with open(os.path.join(tb3, 'urdf', 'turtlebot3_' + model + '.urdf'), encoding='utf-8') as source:
        robot_description = source.read()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        include(gazebo, 'gzserver.launch.py',
                {'world': os.path.join(share, 'worlds', 'project1_maze.world')}),
        include(gazebo, 'gzclient.launch.py', {}),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        include(os.path.join(tb3, 'launch'), 'spawn_turtlebot3.launch.py',
                {'x_pose': '0.6', 'y_pose': '0.35'}),
        include(os.path.join(share, 'launch'), 'wall_follower.launch.py',
                {'use_sim_time': 'true', 'stamped_cmd_vel': 'false'}),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'config', 'project1_ros2.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
