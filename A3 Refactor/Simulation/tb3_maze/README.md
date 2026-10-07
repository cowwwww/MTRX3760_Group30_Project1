# A3 — final refactored Gazebo simulation

Use this package for the final simulation demo. The pre-refactor A1 package is
preserved under `A1 Maze Simulation/tb3_maze` for comparison.

See [A3 build and launch instructions](../../README.md). They build this package
and the existing vendor dependencies using explicit source paths, with output
in `~/tb3_a3_build`. Do not build the historical and final `tb3_maze` together.
The package name, executable, launch files, world and right-wall behaviour stay
unchanged. Use Jazzy, Gazebo Harmonic and `TURTLEBOT3_MODEL=burger_cam`.

The convenience scripts `run_sim.sh` and `tb3_env.sh` use that A3 build directory.
From this directory, after setting the model:

```bash
./run_sim.sh --build
```

In a second terminal, source `tb3_env.sh` and run
`ros2 run tb3_maze turtlebot3_drive`. It moves when scans arrive. The physical
controller is the separate `A3 Refactor/Physical ROS` package.

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
| `CRightWallFollowerRobot` | `src/c_right_wall_follower_robot.cpp`, `include/.../c_right_wall_follower_robot.h` | Right-hand maze strategy. Implements only `ChooseWaypoint()`. |

**To add another maze algorithm**, derive from `CRobot`, implement `ChooseWaypoint()` (read `GetLidar()`, put a point in the robot's frame in `arTarget`, and return true), and construct it in the `Turtlebot3Drive` constructor. Nothing else changes.

**What `CRightWallFollowerRobot` does** (the right-hand rule, with no wall fitting). It finds the gaps in the scan within 70 degrees of straight ahead: directions open for at least 0.90 m and wide enough for the robot. It takes the **rightmost** gap and puts a waypoint 0.5 m away, aimed inside that gap's right edge. The margin keeps 0.25 m from the obstacle at that edge and is capped at the gap midpoint so a narrow opening cannot be aimed across.
- **Along a wall on the right:** it follows that boundary and corrects clearance.
- **At an opening on the right:** it aims into that opening.
- **Where the corridor turns left:** it takes the available left passage.
- **At a dead end:** it searches the rear sector and prefers its rightmost gap.

The forward gap list is ordered from right to left, so the first gap wins. The rear sector spans +70 to +290 degrees, running from the left through the back to the right, so the last gap wins there. The selected right-edge angle is increased by the clearance margin; a positive angular command is still a left turn under ROS conventions. This changes the steering decisions, not just the class name.

Build and run the direction regression checks:

```bash
colcon --log-base ~/tb3_a3_build/log test --build-base ~/tb3_a3_build/build \
  --install-base ~/tb3_a3_build/install --packages-select tb3_maze --event-handlers console_direct+
colcon test-result --test-result-base ~/tb3_a3_build/build --verbose
```

These checks cover competing openings, right-edge clearance and dead-end ordering. They do not replace a complete Gazebo maze run with LiDAR/camera/trajectory evidence.

**How a waypoint is kept.** The algorithm is asked for one on every scan, but the current waypoint is only replaced when the new one is more than 0.25 m from it, or the robot has reached it (within 0.12 m). That keeps the waypoints from creeping forward with the robot and keeps the history readable. With no waypoint at all the robot turns on the spot to the right.

**How it drives to one.** It turns toward it in proportion to the angle (1.5 rad/s per radian, 1.2 rad/s at most), drives at up to 0.15 m/s scaled down by how far off to one side the waypoint is, does not move forward at all when the waypoint is more than 70 degrees off to a side, and slows to a stop for anything within 0.5 m to 0.18 m straight ahead.

Motion constants are in `src/c_robot.cpp`; strategy constants are in `src/c_right_wall_follower_robot.cpp`.

**Invalid scans and recovery.** A scan needs finite metadata, a positive angular
step, valid range limits, and usable data in the front sector (20 degrees either
side of straight ahead). The sector requires at least 70% of its expected rays,
a ray within 10 degrees of its centre, and at least 60% usable readings among
the rays present. These tolerances allow isolated dropouts while rejecting
missing or mostly corrupt front data. They are implementation choices, not
additional assignment marking requirements.

A usable reading is a positive finite distance within the reported range limits,
or positive infinity meaning no obstacle returned within range. NaN, negative
infinity, zero and out-of-range finite values are invalid. When the scan or front
coverage is unusable, both wheels stop and the waypoint history is cleared.
The next usable scan chooses a fresh target from that scan and the latest pose;
the robot does not resume its pre-fault waypoint. Valid open-space scans still
permit movement. This check runs when a scan arrives; it does not add a timeout
for scans that cease arriving.

**Conventions.** Angles are in degrees counted counter-clockwise from the robot's heading, as ROS does (+90 is left, -90 is right). A positive turn rate is a left turn. Wheel speeds are in m/s, limited to 0.22 m/s, and the wheel base is 0.160 m. Waypoints are in the `odom` frame.
