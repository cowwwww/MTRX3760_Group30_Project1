# A3 — final physical right-wall controller

The tested deployment uses ROS 2 Jazzy. See `RUNNING.txt` for workspace and launch
commands. The refactor preserves the node/executable names, topic names, QoS,
parameter names/defaults, 20 Hz control loop and shutdown stop sequence.

| Component | Responsibility | Files |
|---|---|---|
| `LaserScan`, `Velocity` | Plain input/output values independent of ROS | `include/project1/ControlTypes.h` |
| `Settings` | Controller configuration and validation | `include/project1/Settings.h`, `src/Settings.cpp` |
| `ScanProcessor` | Angular sectors, usable rays and distance aggregation | `include/project1/ScanProcessor.h`, `src/ScanProcessor.cpp` |
| `WallFollower` | Control state, corner hysteresis and right-wall steering | `include/project1/WallFollower.h`, `src/WallFollower.cpp` |
| `ros2::WallFollowerNode` | Parameter loading, subscriptions and coordination | `include/project1/ros2/WallFollowerNode.h`, `src/ros2/WallFollowerNode.cpp` |
| `ros2::CommandPublisher` | Twist/TwistStamped selection, command frame and stamping | `include/project1/ros2/CommandPublisher.h`, `src/ros2/CommandPublisher.cpp` |
| `ros2::TrajectoryRecorder` | Odometry filtering and bounded path history | `include/project1/ros2/TrajectoryRecorder.h`, `src/ros2/TrajectoryRecorder.cpp` |
| ROS 2 entry point | Signals, tick timing and stop-before-shutdown lifecycle | `src/turtlebot3_drive_ros2.cpp` |

The node owns its controller, command publisher and trajectory recorder by value;
their lifetime follows the node. `CommandPublisher` refers to that node for ROS
publication and time. The recorder accepts ROS messages without requiring a
running node, so path limits and filtering can be checked directly.

The portable controller is also used by the retained ROS 1 adapter. ROS 1 code is
unchanged and has not been built or run during the Jazzy refactor validation.
A1 Gazebo uses a separate waypoint controller; it does not call this `WallFollower`.

For regression tests, before/after comparisons and the A3 design explanation,
see `../../validation/A3_REFACTOR.md`.
