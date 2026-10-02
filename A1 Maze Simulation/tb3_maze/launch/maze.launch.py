#!/usr/bin/env python3
#
# Launches a maze in Gazebo with the TurtleBot3 spawned inside it.
#
#     ros2 launch tb3_maze maze.launch.py                    the default maze
#     ros2 launch tb3_maze maze.launch.py world:=maze_2      another maze: any world in tb3_maze/worlds
#     ros2 launch tb3_maze maze.launch.py x_pose:=-1.0 y_pose:=0.5
#
# Adapted from turtlebot3_gazebo/launch/turtlebot3_world.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Where the robot starts in each maze (x, y in metres): the centre of the start cell,
# on the west side of the maze, facing +x. A new maze needs a line here, or x_pose and y_pose.
SPAWN_POSES = {
    'default_maze': (-2.0, -1.3),
    'maze_1': (-1.6, 0.0),
    'maze_2': (-2.8, 1.6),
    'maze_3': (-2.4, -2.4),
}


def launch_setup(context):
    # Third party: robot model, bridges, robot_state_publisher.
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_dir = os.path.join(tb3_gazebo_dir, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Ours: the maze world.
    world_name = LaunchConfiguration('world').perform(context)
    worlds_dir = os.path.join(get_package_share_directory('tb3_maze'), 'worlds')
    world = os.path.join(worlds_dir, world_name + '.world')

    if not os.path.isfile(world):
        available = sorted(f[:-len('.world')] for f in os.listdir(worlds_dir) if f.endswith('.world'))
        raise RuntimeError(
            "No maze called '{}'. Available: {}. A new world file needs a rebuild "
            "(./run_sim.sh --build).".format(world_name, ', '.join(available)))

    # The start position for this maze, unless x_pose and y_pose were given.
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    default_x, default_y = SPAWN_POSES.get(world_name, (0.0, 0.0))
    x_pose = x_pose if x_pose else str(default_x)
    y_pose = y_pose if y_pose else str(default_y)

    use_sim_time = LaunchConfiguration('use_sim_time')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # The robot's model files are found through the third-party models folder.
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models'))

    return [
        gzserver_cmd,
        gzclient_cmd,
        spawn_turtlebot_cmd,
        robot_state_publisher_cmd,
        set_env_vars_resources,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='default_maze',
            description='Which maze: the name of a world file in tb3_maze/worlds'),
        DeclareLaunchArgument(
            'x_pose', default_value='',
            description="Start x in metres; empty uses the maze's own start position"),
        DeclareLaunchArgument(
            'y_pose', default_value='',
            description="Start y in metres; empty uses the maze's own start position"),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock'),
        OpaqueFunction(function=launch_setup),
    ])
