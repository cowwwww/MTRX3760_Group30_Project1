# Optional A3 physical controller: focused A2 revision

ROS package: `mtrx3760_wall_follow_trial`. Node: `project1_wall_follower_trial`.
This separate identity lets the team retain its existing implementation.
The control core and parameter values are unchanged from the assessed handoff.

This package retains the original `Settings`, `ScanProcessor`, `WallFollower`,
`WallFollowerNode`, `CommandPublisher` and `TrajectoryRecorder` classes. The
ordinary wall-following equation, gains, 20 Hz loop, topics and ownership remain.
`ScanProcessor` now supplies body clearance and evidence of an opening;
`WallFollower` adds two corner states and uses odometry feedback. `FindWall` now
holds the new heading while reacquiring a nearby wall, instead of circling.

This is a functional correction within A2's architecture, as well as an OO
refactor. It is not a behaviour-preserving refactor. No mapping, route planner,
line-fitting class, SLAM, new ROS node or hardware change is introduced.

See [the trial guide](../README.md) for isolated build/test commands, recording
and switching back, and [the assessment](../ASSESSMENT.md) for the control
reasoning and known failures.
The ROS 2 adapter was built on Jazzy. The retained ROS 1 adapter was updated for
odometry input but has not been built here. The separate simulation package is
unchanged and must still satisfy A1 independently.
