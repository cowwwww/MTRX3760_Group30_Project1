# MTRX3760 Project 1 — Group 30

One ROS 2 program, `mtrx3760_project1/turtlebot3_drive`, performs sensor-driven
right-wall following in Gazebo and on the physical TurtleBot3. Both launch paths
use the same `WallFollowerNode`, `WallFollower`, settings file and executable.
The simulation launch adds the maze, simulated sensors and visualisation.

## Repository layout

- `src/mtrx3760_project1/`: the final controller, tests, configuration, launch files,
  maze worlds and RViz configuration.
- `third_party/`: one copy of the TurtleBot3 description and Gazebo dependencies,
  including the camera-equipped Burger model used for A1.
- `Report_ROS/`: editable design diagram sources.
- `validation/`: test tools and dated historical evidence. Historical results are
  not evidence of a new revision; see `docs/CONSOLIDATION.md`.
- `docs/`: consolidation provenance, validation status and submission checklist.

Earlier A1, A2, A3 and experiment implementations are available through Git
history, not parallel source folders in the final checkout. No history was rewritten.

## Build (ROS 2 Jazzy)

Use a fresh terminal. Build outside the repository to keep generated output out
of the submission. The machine needs Jazzy, colcon, ros_gz, RViz, robot state
publisher, the message dependencies in package.xml, and Cyclone DDS for the tests.

```bash
cd /home/james/mtrx3760/MTRX3760_Group30_Project1
source /opt/ros/jazzy/setup.bash
colcon --log-base /home/james/mtrx3760_ws/consolidated/log build \
  --base-paths src third_party \
  --build-base /home/james/mtrx3760_ws/consolidated/build \
  --install-base /home/james/mtrx3760_ws/consolidated/install \
  --cmake-args -DBUILD_TESTING=ON
source /home/james/mtrx3760_ws/consolidated/install/setup.bash
```

## Simulation (A1)

```bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch mtrx3760_project1 maze.launch.py
```

This starts Gazebo, LiDAR, camera, the shared wall follower and RViz. The robot
starts beside its right wall in `default_maze`. Other worlds can be selected with
`world:=maze_1` (or `maze_2`, `maze_3`); they require separate validation. Use
`gui:=false rviz:=false` for headless operation, or `controller:=false` when a
validation tool starts the controller itself. RViz displays the `/trajectory`
path recorded from odometry, along with `/scan` and `/camera/image_raw`.

## Physical robot (A2)

On the robot, use its installed hardware drivers:

```bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-04
ros2 launch turtlebot3_bringup robot.launch.py
```

In a second terminal with the built workspace sourced and the same ROS domain
and middleware as the robot:

```bash
ros2 launch mtrx3760_project1 wall_follower.launch.py
```

The documented hardware is Burger with LDROBOT LD19 (`LDS_MODEL=LDS-04`). The
controller consumes `/scan` and `/odom`, publishes stamped `/cmd_vel`,
`/trajectory` and `/controller_diagnostics`. `stamped_cmd_vel:=false` supports a
platform expecting `Twist`. The default is the demonstrated physical settings;
`use_sim_time:=true` is set by the simulation launch. Configuration is loaded at
startup; restart the node after changing controller settings.

## Tests

```bash
ctest --test-dir /home/james/mtrx3760_ws/consolidated/build/mtrx3760_project1 --output-on-failure
python3 validation/test_maze_exit.py
python3 validation/gazebo_maze_check.py --output /tmp/group30-maze --timeout 500
```

The Gazebo checker uses a separate localhost ROS domain and Gazebo partition,
records camera images and trajectory, and checks the actual east exit crossing.
For core tests without ROS, configure with `-DPROJECT1_ROS_VERSION=`.
Read `docs/CONSOLIDATION.md` for actual results and unresolved failures.

## Submission

Follow `docs/SUBMISSION_CHECKLIST.md`. Commit the reviewed final changes before
producing the Git log, source appendix and full-history bundle. Physical
functionality is assessed by the live tutor demonstration, not offline tests.
