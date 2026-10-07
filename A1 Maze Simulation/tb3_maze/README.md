# TurtleBot3 Maze Simulation: How It Works and How to Launch It

Uses: **ROS 2 Jazzy** and **Gazebo Harmonic** (`gz sim` 8.x).

## Folder layout

Our code and the third-party code are kept apart, for longevity.

```
MTRX3760 - Project 1/            the workspace: colcon builds from here
├── tb3_maze/                  OUR code: the ROS package tb3_maze tb3_env.sh, run_sim.sh, README.md
└── tb3_third_parties/         ROBOTIS TurtleBot3 packages, kept unmodified
```

- Never edit `tb3_third_parties`. `tb3_maze` depends on the third-party `turtlebot3_gazebo` and includes its launch files for the robot, the bridges and the state publisher.
- To change something that is third-party, put a copy in `tb3_maze` and change that. The drive node, the maze world, the launch file and the RViz config were done this way.

## Prereqs

Need to install ros 2 jazzy packages:
```bash
# Gazebo/ROS bridge packages
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-robot-state-publisher ros-jazzy-rqt-image-view
```

## Initial setup

Build from the workspace folder. `colcon` finds the packages below it. Two are needed for the simulation: the third-party `turtlebot3_gazebo` and our `tb3_maze`. The build, install and log output goes to a folder outside the project, in this case **`~/tb3_build`** (see the next section), not into the source folders:

```bash
cd "/home/ubuntu/Documents/MTRX3760 - Sandbox"
source /opt/ros/jazzy/setup.bash
mkdir -p ~/tb3_build
colcon --log-base ~/tb3_build/log build --symlink-install --packages-select turtlebot3_gazebo tb3_maze \
  --build-base ~/tb3_build/build --install-base ~/tb3_build/install
```

`./run_sim.sh --build` runs exactly this.

Note: Need to re-run this `colcon build` command after every code change, and after **adding** a new file (a new world, model, launch file or parameter file).

### What `~/tb3_build` is

It's a build output folder. The source stays in the Project folder, and `~/tb3_build` holds what `colcon build` produces. It is deleted and rebuilt freely, and it should never be committed or submitted.

| Folder | Contents | Used by |
|---|---|---|
| `~/tb3_build/build/` | Intermediate files (CMake cache, object files, the `turtlebot3_drive` binary before install) | colcon only |
| `~/tb3_build/install/` | What ROS actually runs: for each package the executable in `lib/<package>/` and `share/<package>/` (launch files, worlds, models, params, URDFs), and the `setup.bash` you source | ROS, Gazebo, you |
| `~/tb3_build/log/` | colcon's build logs | you, when a build fails |

**Why it is not in the source folder.** `source install/setup.bash` adds the install folder to `AMENT_PREFIX_PATH`, and the launch file builds the world's path from it with `get_package_share_directory('tb3_maze')`. 

That path is passed to `gz sim` as one string and Gazebo **splits it on spaces**. With the install folder under `MTRX3760 - Sandbox` (example), the world path breaks and Gazebo cannot find the world and dies. 

A "space-free" install folder avoids this and renaming the source folder without spaces would also work.

## Quick Start (Scripts)

Two scripts in this folder replace the setup commands:

| Script | What it does |
|---|---|
| `run_sim.sh` | Sources everything and launches a world. Run it, don't source it. |
| `tb3_env.sh` | Sets up a terminal (ROS, `~/tb3_build`, `TURTLEBOT3_MODEL`). Source it, don't run it. |

`source tb3_env.sh` just runs:
```bash
source /opt/ros/jazzy/setup.bash
source ~/tb3_build/install/setup.bash
export TURTLEBOT3_MODEL=burger_cam
```

### Terminal 1 - Gazebo

This starts the gz server, the gz window, `robot_state_publisher`, the robot spawn, `ros_gz_bridge` and the image bridge. In the test maze the robot spawns at (-2.0, -1.3) facing +x.

```bash
cd "/home/<your path>/tb3_maze"
run_sim.sh
```

Some options are:

```bash
./run_sim.sh                         # launch the default maze (Terminal 1)
./run_sim.sh turtlebot3_world        # launch another world: any launch file in tb3_maze/launch or turtlebot3_gazebo/launch
./run_sim.sh --build                 # rebuild first, then launch
./run_sim.sh --build turtlebot3_world x_pose:=-2.0 y_pose:=-0.5    # extra arguments go to ros2 launch
```

### Terminal 2 - RViz

```bash
cd "/home/<your path>/tb3_maze"
source tb3_env.sh
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix tb3_maze)/share/tb3_maze/rviz/tb3_maze.rviz"
```

What the command does:
- `-d <file>` RViz option that loads a display config file at startup
- `$(ros2 pkg prefix tb3_maze)` Substitutes in the tb3_maze install path prefix
- `/share/tb3_maze/rviz/tb3_maze.rviz` Config file path inside tb3_maze

### Terminal 3: wall follower / driver node

```bash
cd "/home/<your path>/tb3_maze"
source tb3_env.sh
ros2 run tb3_maze turtlebot3_drive
```

Start it after Terminal 1 is up. The robot drives as soon as the first scan arrives. 

### (optional) Terminal 4: camera view and manual driving 

```bash
source tb3_env.sh
ros2 run rqt_image_view rqt_image_view /camera/image_raw
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```


## RViz Config

- The config has TF, LaserScan, Odometry and RobotModel displays with fixed frame `odom`. The TF display shows the frame axes but not their white name labels
- The Odometry display is the red heading arrow. 
- The robots **Waypoints** are shown as a big cyan ball, and the two before it as smaller, fainter ones.
- **Odometry trail** is a second Odometry display of very small green arrows. Keeps up to 10000 of them, dropping a new one each time the robot moves 5 cm or turns 0.1 rad, so you can see where it has been. 
- **LaserScan history** is a second LaserScan display with the same point size and style as the live one, but orange. It keeps every scan for 600 s (`Decay Time`), so the walls the robot has already passed stay on screen. 
- It has no camera display. Add one with **Add → By topic → /camera/image_raw → Image**.


## The robot control code

The drive code is written in the style of the Lab 2 A2 code: a general base class for each kind of thing, with the parts that differ between algorithms left virtual. It does **not** use the Lab 1 or Lab 2 simulation code. Gazebo replaces the simulator.

The job is split in two: **where to go** (a waypoint) and **how to get there**.

```
lidar scan + pose  ->  ChooseWaypoint()  ->  drive to the waypoint  ->  wheel speeds  ->  /cmd_vel
```

| Class | Files (in `tb3_maze/`) | Role |
|---|---|---|
| `Turtlebot3Drive` | `src/turtlebot3_drive.cpp`, `include/.../turtlebot3_drive.hpp` | ROS node. Scan and odom in, `cmd_vel` and `waypoints` out. Holds a `std::unique_ptr<CRobot>`, so any algorithm can be used here. |
| `CSensor` | `src/c_sensor.cpp`, `include/.../c_sensor.h` | Base for a sensor mounted on the robot. Has the mount angle and a virtual `HasReading()`. |
| `CLidar` | same files | The 360 degree LiDAR. Keeps the latest scan. `FindGaps()` returns the stretches of directions that are open for a given distance, with the distance to whatever ends each one. |
| `CRobot` | `src/c_robot.cpp`, `include/.../c_robot.h` | Base for a maze algorithm. Keeps the waypoints (the current one and the two before it, in the odom frame), turns toward the current one, drives to it, slows for obstacles ahead, and converts that to wheel speeds. |
| `CRightWallFollowerRobot` | same files | The first algorithm. Implements only `ChooseWaypoint()`. |

**To add another maze algorithm**, derive from `CRobot`, implement `ChooseWaypoint()` (read `GetLidar()`, put a point in the robot's frame in `arTarget`, and return true), and construct it in the `Turtlebot3Drive` constructor. Nothing else changes.

**What `CRightWallFollowerRobot` does** (the right-hand rule, with no wall fitting). It finds the gaps in the scan within 70 degrees of straight ahead: directions open for at least 0.90 m and wide enough for the robot. It takes the **rightmost** gap and puts a waypoint 0.5 m away, aimed inside that gap's right edge. The margin keeps 0.25 m from the obstacle at that edge and is capped at the gap midpoint so a narrow opening cannot be aimed across.
- **Along a wall on the right:** it follows that boundary and corrects clearance.
- **At an opening on the right:** it aims into that opening.
- **Where the corridor turns left:** it takes the available left passage.
- **At a dead end:** it searches the rear sector and prefers its rightmost gap.

The forward gap list is ordered from right to left, so the first gap wins. The rear sector spans +70 to +290 degrees, running from the left through the back to the right, so the last gap wins there. The selected right-edge angle is increased by the clearance margin; a positive angular command is still a left turn under ROS conventions. This changes the steering decisions, not just the class name.

Build and run the direction regression checks:

```bash
colcon --log-base ~/tb3_build/log test --build-base ~/tb3_build/build \
  --install-base ~/tb3_build/install --packages-select tb3_maze --event-handlers console_direct+
colcon test-result --test-result-base ~/tb3_build/build --verbose
```

These checks cover competing openings, right-edge clearance and dead-end ordering. They do not replace a complete Gazebo maze run with LiDAR/camera/trajectory evidence.

**How a waypoint is kept.** The algorithm is asked for one on every scan, but the current waypoint is only replaced when the new one is more than 0.25 m from it, or the robot has reached it (within 0.12 m). That keeps the waypoints from creeping forward with the robot and keeps the history readable. With no waypoint at all the robot turns on the spot to the right.

**How it drives to one.** It turns toward it in proportion to the angle (1.5 rad/s per radian, 1.2 rad/s at most), drives at up to 0.15 m/s scaled down by how far off to one side the waypoint is, does not move forward at all when the waypoint is more than 70 degrees off to a side, and slows to a stop for anything within 0.5 m to 0.18 m straight ahead.

All the numbers are constants at the top of each class in `src/c_robot.cpp`.

**Conventions.** Angles are in degrees counted counter-clockwise from the robot's heading, as ROS does (+90 is left, -90 is right). A positive turn rate is a left turn. Wheel speeds are in m/s, limited to 0.22 m/s, and the wheel base is 0.160 m. Waypoints are in the `odom` frame.

## Checking it is working

```bash
ros2 topic list                       # expect /scan /odom /cmd_vel /tf /camera/image_raw ...
ros2 topic hz /scan                   # ~5 Hz
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames        # writes frames.pdf of the TF tree
ros2 run rqt_graph rqt_graph          # see the live node/topic graph (diagnostic only)
gz topic -l                           # list topics on the Gazebo side
```

## Known issues

1. **`cmd_vel` message type (fixed).** The bridge YAMLs map `cmd_vel` to `geometry_msgs/TwistStamped`, but the stock `turtlebot3_drive.cpp` published plain `Twist`, so the robot would not move. Our node now publishes `TwistStamped`. The real robot also expects it (`enable_stamped_cmd_vel: true` in `turtlebot3_bringup/param/burger.yaml`). Any other node you write for `cmd_vel` must do the same.
2. **Only a small test maze exists.** The provided worlds are `turtlebot3_world`, `house`, `dqn_stage1-4`, `autorace` and `empty`. `turtlebot3_maze_test` (in `tb3_maze/worlds`) was added for wall-following tests. Any further maze needs a world file under `tb3_maze/worlds/` and a matching launch file in `tb3_maze/launch/`.
3. **`TURTLEBOT3_MODEL` unset** makes both the spawn and robot-state-publisher launch files crash with a `KeyError`. Export it in every terminal.
4. **Unknown package.** If `ros2` reports a package is not found, `source ~/tb3_build/install/setup.bash` was skipped in that terminal.
5. **Spaces in the install path** break the world launch. The error is `Fuel world download failed ... Unable to find or download file`, with the server dying straight after. This is why the build goes to `~/tb3_build`. Do not build with the default `install/` inside the Sandbox folder.
6. **A leftover Gazebo server breaks the next launch.** If a `gz sim` process survives from an earlier run, the new simulation shares its topics. You then see the old world's robot and sensor data, and spawns can land in the wrong world. Check with `ps -eo pid,etime,args | grep "gz sim"` and clear it with `pkill -f "gz sim"` before launching.

## Where things are

Paths are relative to `MTRX3760 - Sandbox`.

| What | Path |
|---|---|
| Driver node | `tb3_maze/src/turtlebot3_drive.cpp`, `tb3_maze/include/tb3_maze/turtlebot3_drive.hpp` |
| Robot control (`CRobot`, `CRightWallFollowerRobot`, `CSensor`, `CLidar`) | `tb3_maze/src/c_robot.cpp`, `c_sensor.cpp` and `tb3_maze/include/tb3_maze/c_robot.h`, `c_sensor.h` |
| Launch files (ours) | `tb3_maze/launch/` |
| Worlds (test maze: `turtlebot3_maze_test.world`) | `tb3_maze/worlds/` |
| RViz config | `tb3_maze/rviz/tb3_maze.rviz` |
| Launch and terminal setup scripts | `tb3_maze/run_sim.sh`, `tb3_maze/tb3_env.sh` |
| Robot model (LiDAR, camera, DiffDrive), third party | `tb3_third_parties/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf` |
| Gazebo↔ROS topic bridge config, third party | `tb3_third_parties/turtlebot3_simulations/turtlebot3_gazebo/params/` |
| Real-robot driver (for A2), third party | `tb3_third_parties/turtlebot3-main/turtlebot3_node/`, `tb3_third_parties/turtlebot3-main/turtlebot3_bringup/` |




