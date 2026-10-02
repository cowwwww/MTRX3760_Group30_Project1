#!/usr/bin/env bash
# run_sim.sh - launch the TurtleBot3 Gazebo simulation in one command.
#
# Usage:
#     ./run_sim.sh                          launch the default maze
#     ./run_sim.sh maze_2                   launch another maze (any world in tb3_maze/worlds)
#     ./run_sim.sh turtlebot3_world         launch a third-party world (any launch file in turtlebot3_gazebo/launch)
#     ./run_sim.sh --build                  rebuild first, then launch the default maze
#     ./run_sim.sh --build maze_1 x_pose:=-1.0 y_pose:=0.5
#
# A name is looked for as a maze in tb3_maze/worlds first, then as a launch file in
# tb3_maze/launch, then as one in turtlebot3_gazebo/launch.
# Extra arguments after the name are passed to `ros2 launch`.
# Rebuild (--build) after changing C++ or adding a new world, model or launch file.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"     # the workspace: holds tb3_maze and tb3_third_parties
BUILD_DIR="${TB3_BUILD_DIR:-$HOME/tb3_build}"

source "$SCRIPT_DIR/tb3_env.sh"

if [ "$1" = "--build" ]; then
  shift
  mkdir -p "$BUILD_DIR"
  (cd "$WS_DIR" && colcon --log-base "$BUILD_DIR/log" build --symlink-install \
      --packages-select turtlebot3_gazebo tb3_maze \
      --build-base "$BUILD_DIR/build" --install-base "$BUILD_DIR/install") || exit 1
  source "$BUILD_DIR/install/setup.bash"
fi

NAME="${1:-default_maze}"
[ $# -gt 0 ] && shift
NAME="${NAME%.launch.py}"
NAME="${NAME%.world}"

TB3_MAZE_SHARE="$(ros2 pkg prefix tb3_maze 2>/dev/null)/share/tb3_maze"

if [ -f "$TB3_MAZE_SHARE/worlds/$NAME.world" ]; then
  exec ros2 launch tb3_maze maze.launch.py world:="$NAME" "$@"
elif [ -f "$TB3_MAZE_SHARE/launch/$NAME.launch.py" ]; then
  exec ros2 launch tb3_maze "$NAME.launch.py" "$@"
elif [ -f "$SCRIPT_DIR/worlds/$NAME.world" ]; then
  echo "run_sim.sh: '$NAME' is a new world that has not been built yet. Run: ./run_sim.sh --build $NAME" >&2
  exit 1
else
  exec ros2 launch turtlebot3_gazebo "$NAME.launch.py" "$@"
fi
