# Optional A2 wall-following trial

This folder adds an alternative physical controller without replacing the
group's implementation. It uses ROS package `mtrx3760_wall_follow_trial`, node
`project1_wall_follower_trial`, and executable `turtlebot3_drive`. The separate
package identity prevents ROS package lookup from selecting `mtrx3760_project1`
when the trial is requested. The existing class/namespace names remain.

The controller core and parameter values are unchanged from the assessed
`turtlebot_a2_refactor_20261008` handoff. Only package/node names and their launch,
configuration and diagnostic references have changed. [ASSESSMENT.md](ASSESSMENT.md)
explains the control approach, limits and two remaining synthetic navigation
failures. This controller has not been physically validated.

The experiment root contains `COLCON_IGNORE` to keep this addition out of normal
recursive repository builds. Copy the inner package as instructed below; do not
copy the whole experiment into a ROS workspace. This follows
[colcon's package-discovery rules](https://colcon.readthedocs.io/en/released/reference/discovery-arguments.html).

## Build on the university desktop

Use a fresh terminal with ROS Jazzy sourced. Do not source the team's old overlay
in this build terminal. The checkout and workspace below are newly created, so
they cannot overwrite an existing build or checkout.

```bash
source /opt/ros/jazzy/setup.bash
TB3_TRIAL_CHECKOUT=$(mktemp -d "$HOME/mtrx3760_wall_trial_checkout_XXXXXX")
git clone --branch codex/a2-wall-follow-trial --single-branch \
  https://github.com/cowwwww/MTRX3760_Group30_Project1.git "$TB3_TRIAL_CHECKOUT"
TB3_TRIAL_WS=$(mktemp -d "$HOME/mtrx3760_wall_trial_ws_XXXXXX")
mkdir -p "$TB3_TRIAL_WS/src"
cp -a "$TB3_TRIAL_CHECKOUT/experiments/a2_wall_follow_trial/mtrx3760_wall_follow_trial" \
  "$TB3_TRIAL_WS/src/"
printf 'export TB3_TRIAL_WS=%q\nexport TB3_TRIAL_CHECKOUT=%q\n' \
  "$TB3_TRIAL_WS" "$TB3_TRIAL_CHECKOUT" > "$HOME/tb3_wall_trial_workspace.env"
cd "$TB3_TRIAL_WS"
colcon list --base-paths src
colcon build --base-paths src --packages-select mtrx3760_wall_follow_trial \
  --cmake-args -DPROJECT1_ROS_VERSION=2 -DBUILD_TESTING=ON
```

`colcon list` should show only `mtrx3760_wall_follow_trial`. These commands assume
the established Jazzy dependencies are installed: `rclcpp`, `sensor_msgs`,
`geometry_msgs`, `nav_msgs`, `diagnostic_msgs`, `launch_ros`, `ament_index_python`,
`rclpy` and Cyclone DDS. If a dependency is missing, preserve the build output
before diagnosing it. No robot-side software replacement is needed to run the
controller on the desktop.

In another fresh terminal, run the offline checks:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/tb3_wall_trial_workspace.env"
source "$TB3_TRIAL_WS/install/local_setup.bash"
cd "$TB3_TRIAL_WS"
colcon test --base-paths src --packages-select mtrx3760_wall_follow_trial \
  --event-handlers console_direct+
colcon test-result --verbose
ros2 pkg prefix mtrx3760_wall_follow_trial
```

The prefix must be `$TB3_TRIAL_WS/install/mtrx3760_wall_follow_trial`. The ROS
integration test sets loopback-only discovery, domain 201 and private
`/group30_offline/...` topics internally. It does not command the physical robot.
These checks do not replace a Gazebo or physical demonstration.

## Connect to the established robot

The established robot bringup stays in use. If it is already running, do not
launch it again. If it is not running, use its normal setup on the robot:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-04
ros2 launch turtlebot3_bringup robot.launch.py
```

On the desktop, use the following environment in every physical-test terminal:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/tb3_wall_trial_workspace.env"
source "$TB3_TRIAL_WS/install/local_setup.bash"
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
ros2 pkg prefix mtrx3760_wall_follow_trial
ros2 topic info /scan --verbose
ros2 topic info /odom --verbose
ros2 topic info /cmd_vel --verbose
```

Stop the old wall follower, navigation controller and any active teleoperation
publisher using their own terminals. `/cmd_vel` should have no competing command
publisher. Checking the node name alone is insufficient; inspect the publishers
shown by `ros2 topic info /cmd_vel --verbose`.

Check the stationary laser display with `rviz2 -f base_link`, adding `/scan` with
Best Effort reliability. Confirm forward/right directions, the actual mounting
and the body envelope against the YAML settings. Check both machines' clocks with
`timedatectl status` and `date --iso-8601=ns`; stale/future header stamps cause a
named hold. The range target is 0.25 m from the laser, approximately 0.16 m body
side clearance for the assumed envelope, not 0.25 m body clearance.

For a stationary diagnostic launch, route commands to a separate topic first:

```bash
ros2 launch mtrx3760_wall_follow_trial wall_follower.launch.py \
  stamped_cmd_vel:=true cmd_vel_topic:=/wall_trial_cmd_vel_check
```

In another prepared terminal:

```bash
ros2 param get /project1_wall_follower_trial wall_distance
ros2 topic echo /controller_diagnostics
```

The wall-distance parameter should be `0.25`. Stop this launch with Ctrl+C before
starting the actual trial. The remapped diagnostic launch does not feed `/cmd_vel`.

## Record the physical trial

Prepare desktop terminal A with the physical-test environment above, then:

```bash
TB3_RUN=$(mktemp -d "$HOME/tb3_wall_trial_$(date +%Y%m%d_%H%M%S)_XXXXXX")
printf 'export TB3_RUN=%q\n' "$TB3_RUN" > "$HOME/tb3_wall_trial_run.env"
git -C "$TB3_TRIAL_CHECKOUT" rev-parse HEAD > "$TB3_RUN/repository_commit.txt"
cp -a "$TB3_TRIAL_WS/src/mtrx3760_wall_follow_trial" "$TB3_RUN/source"
ros2 pkg prefix mtrx3760_wall_follow_trial > "$TB3_RUN/package_prefix.txt"
sha256sum "$TB3_TRIAL_WS/install/mtrx3760_wall_follow_trial/lib/mtrx3760_wall_follow_trial/turtlebot3_drive" \
  > "$TB3_RUN/executable.sha256"
cat > "$TB3_RUN/layout_notes.txt" <<'NOTES'
Corridor width; wall endpoints; outgoing length; end wall / next opening:
Starting body clearance, orientation and distance before corner:
Wall material/height and stationary RViz observations:
Interventions/contact, including approximate time:
Success criterion and distance followed after the turn:
Video filename and time reference, if available:
NOTES
ros2 bag record -o "$TB3_RUN/bag" /scan /odom /cmd_vel \
  /controller_diagnostics /trajectory /tf /tf_static /battery_state /rosout /parameter_events
```

Edit the layout notes with actual measurements. The command uses rosbag2's
configured default storage plugin and does not require a specific MCAP plugin.

Prepare terminal B with the same environment, then launch movement:

```bash
source "$HOME/tb3_wall_trial_run.env"
printf '%s\n' 'ros2 launch mtrx3760_wall_follow_trial wall_follower.launch.py stamped_cmd_vel:=true' \
  > "$TB3_RUN/launch_command.txt"
set -o pipefail
ros2 launch mtrx3760_wall_follow_trial wall_follower.launch.py \
  stamped_cmd_vel:=true 2>&1 | tee "$TB3_RUN/controller.log"
```

In prepared terminal C, capture which parameters and publishers actually ran:

```bash
source "$HOME/tb3_wall_trial_run.env"
ros2 param dump /project1_wall_follower_trial > "$TB3_RUN/effective_parameters.yaml"
ros2 node info /project1_wall_follower_trial > "$TB3_RUN/node_info.txt"
ros2 topic info /cmd_vel --verbose > "$TB3_RUN/cmd_vel_publishers.txt"
ros2 topic echo /controller_diagnostics
```

Stop the controller with Ctrl+C, confirm the robot stops, then stop recording.
Keep the full directory even if the trial fails. Do not move walls during a
recorded comparison; record any intervention and start a separate trial.

## Switch back to the team's implementation

Stop the trial controller first. Use a fresh terminal, source Jazzy and the
team's existing workspace, and use their existing launch command. If that is the
`mtrx3760_project1` package, check `ros2 pkg prefix mtrx3760_project1` points to
their workspace before launching. No deletion, rebuilding, Git reset or rollback
is required. Keep the robot bringup running; switch only the command source.

## Rerun the synthetic scenario suite

The included suite contains the unchanged controller cores and 50 recorded runs.
From `simulation/`:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
python3 run_suite.py --build build --output rerun-results
```

Results: 22 new-controller goals completed, one persistent blind hold/timeout,
and two navigation failures. All eleven dead-end variants returned, but the
turnaround was not a smooth constant-clearance semicircle. See
[simulation/README.md](simulation/README.md) for the matrix and assumptions.
Physical wall materials, wheel slip and LiDAR driver behaviour are not simulated.

The project still requires its original A1/A2 demonstrations. Acknowledge AI
assistance as required by the brief. This optional experiment is not a certified
replacement for the group's final implementation.
