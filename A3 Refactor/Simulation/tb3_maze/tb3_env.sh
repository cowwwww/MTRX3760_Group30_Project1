#!/usr/bin/env bash
# tb3_env.sh - set up a terminal for the TurtleBot3 simulation.
#
# Must be SOURCED, not executed, so the settings stay in your terminal:
#     source tb3_env.sh
#
# Does the three things every terminal needs: loads ROS 2 Jazzy, loads the workspace
# built into ~/tb3_a3_build, and selects the robot model.

TB3_BUILD_DIR="${TB3_BUILD_DIR:-$HOME/tb3_a3_build}"

source /opt/ros/jazzy/setup.bash

if [ -f "$TB3_BUILD_DIR/install/setup.bash" ]; then
  source "$TB3_BUILD_DIR/install/setup.bash"
else
  echo "tb3_env.sh: $TB3_BUILD_DIR/install/setup.bash not found. Build first: ./run_sim.sh --build (from tb3_maze)" >&2
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger_cam}"
