# Packaging validation — 9 October 2026

This is a packaging change to the previously assessed controller. It is not an
additional control patch.

- Built the separately named package with local ROS 2 Jazzy and `colcon`, using
  `-DPROJECT1_ROS_VERSION=2 -DBUILD_TESTING=ON`.
- Passed all 26 controller/trajectory CTest entries.
- Passed the ROS adapter integration CTest entry, which contains seven message
  tests, on loopback only and domain 201. The first run in the restricted sandbox
  could not enumerate DDS interfaces; the integration test passed after loopback
  socket access was enabled. No controller source change was needed.
- Launched the installed package on domain 201 with private scan/odom/command
  topics. Verified node `project1_wall_follower_trial`, `wall_distance=0.25`,
  `forward_speed=0.08`, `stamped_cmd_vel=true`, and a private `TwistStamped`
  command publisher. The YAML's renamed node key is therefore actually applied.
- Compared all seven portable controller files byte-for-byte with the assessed
  `turtlebot_a2_refactor_20261008` handoff. All are unchanged. ROS 2 YAML parameter
  values are also unchanged. The renamed files are listed in `source_manifest.json`.
- Verified `colcon list` recursively scanning the experiment root discovers no
  packages because of its `COLCON_IGNORE` marker. The inner ROS package builds
  when copied into its separate workspace.
- The team's original checkout remains on `main` with no local changes.

The included broader scenario suite is unchanged: 22 new-controller goals
completed, one persistent blind hold/timeout, two navigation failures. Passing
the package checks does not erase those navigation failures. No physical TurtleBot
test was performed. The retained ROS 1 adapter has not been built in this check.
