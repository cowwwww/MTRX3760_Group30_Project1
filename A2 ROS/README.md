# A2 — original physical implementation

This is the pre-refactor implementation from commit `ed92004`, retaining the
Jazzy linking fix and offline tests. Its controller source/configuration is
preserved for comparison and the report.

**For the final demo, build `A3 Refactor/Physical ROS` instead.** See the
repository README and `A3 Refactor/README.md`. A3 retains the same ROS package
name, executable, launch arguments and controller parameters.

`COLCON_IGNORE` prevents automatic discovery of this historical copy alongside
A3. To deliberately reproduce the old version, copy this directory into a
separate workspace and remove the copied marker there before building. Do not
source the old and final workspaces together.
