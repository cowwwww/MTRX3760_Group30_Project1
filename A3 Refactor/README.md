# A3 — final refactored implementation

Use **Physical ROS** for the real TurtleBot and **Simulation/tb3_maze** for
Gazebo. These are separate right-wall implementations. Original A1/A2 code
is preserved outside this folder at its pre-refactor version (`ed92004`).

Package names, executable names, launch arguments, controller parameters and
control rules are unchanged. The folder organisation identifies the assignment
stages; it does not add a third controller that runs alongside A1/A2.

## Real TurtleBot — PC/laptop terminal

Stop any running wall follower before rebuilding. Run from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/mtrx3760_ws/src/mtrx3760_project1
cp -a "A3 Refactor/Physical ROS/." ~/mtrx3760_ws/src/mtrx3760_project1/
cd ~/mtrx3760_ws
colcon build --symlink-install --packages-select mtrx3760_project1
source install/setup.bash
ros2 pkg executables mtrx3760_project1
ros2 launch mtrx3760_project1 wall_follower.launch.py --show-args
```

Copy the complete A3 package, including its new headers and source files. If
using a different workspace, substitute its path consistently. Do not copy
`A2 ROS` for the final demo. If that historical package was copied after this
layout change, remove its copied `COLCON_IGNORE` only when intentionally building
that old version; for the final code, use a workspace containing the A3 package.

Match the laptop's ROS distro, middleware and domain to the actual robot. The
confirmed PC and TB3-30 use Jazzy and Cyclone DDS. Match the domain used by the
running robot drivers; do not assume home and KIRBYBOTS use the same domain.
Bring up `turtlebot3_bringup robot.launch.py` on the robot separately, then inspect
live `/scan`, `/odom` and the base **subscriber** on `/cmd_vel`.

After safe manual positioning, stop teleop. Only one motion publisher should be
active. A3 moves when valid scans arrive. On the PC/laptop, once ready to drive:

```bash
ros2 launch mtrx3760_project1 wall_follower.launch.py \
  use_sim_time:=false stamped_cmd_vel:=true \
  scan_topic:=/scan odom_topic:=/odom cmd_vel_topic:=/cmd_vel
```

Use `stamped_cmd_vel:=true` only if the base subscriber accepts
`geometry_msgs/msg/TwistStamped`; use `false` for `geometry_msgs/msg/Twist`.
Verify a straight right wall and stopping, then a corner, then the constructed
maze. Record the actual build revision, YAML, ROS setup, results and interventions.
The controller assumes LiDAR yaw zero unless configured; it does not rotate
scans using TF. Tune only in response to an observed physical problem.

## Gazebo — fresh PC terminal

Build into a separate directory from old A1 builds to avoid stale CMake paths.
From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger_cam
colcon --log-base ~/tb3_a3_build/log build --symlink-install \
  --base-paths "A3 Refactor/Simulation/tb3_maze" \
    "A1 Maze Simulation/tb3_third_parties/turtlebot3_simulations/turtlebot3_gazebo" \
    "A1 Maze Simulation/tb3_third_parties/turtlebot3-main/turtlebot3_description" \
  --packages-select turtlebot3_description turtlebot3_gazebo tb3_maze \
  --build-base ~/tb3_a3_build/build --install-base ~/tb3_a3_build/install
source ~/tb3_a3_build/install/setup.bash
ros2 launch tb3_maze maze.launch.py
```

Leave Gazebo running. In a second fresh terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/tb3_a3_build/install/setup.bash
export TURTLEBOT3_MODEL=burger_cam
ros2 run tb3_maze turtlebot3_drive
```

The convenience scripts in `Simulation/tb3_maze` also use `~/tb3_a3_build` and
the explicit source paths above. Set `TURTLEBOT3_MODEL=burger_cam` before using
them if your shell defaults to `burger`.

## Tests and report

See [the A3 design and evidence](../validation/A3_REFACTOR.md). Existing recorded
results identify the exact revisions tested, including their earlier paths.
Moving identical code into A3 does not turn those records into new robot tests.

For a fresh exact-output comparison, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
python3 validation/compare_controller_revisions.py \
  --baseline ed92004 --candidate HEAD --output /tmp/group30-a3-layout-comparison
```

Use `Report ROS/A3 physical classes.puml` and `A3 simulation classes.puml` for
the final class designs. The final demonstrated revision must be physically
tested and used for the report, source appendix and Git-history export.
