# MTRX3760 Project 1 — Group 30

**Build and demonstrate the final code in `A3 Refactor`.** A1/A2 retain the
implementations from before the structural refactor so the stages are clear.

| Folder | Purpose | Use for final demo? |
|---|---|---|
| `A1 Maze Simulation/tb3_maze` | Original right-wall simulation, before A3 | Reference only |
| `A2 ROS` | Original physical right-wall controller, before A3 | Reference only |
| `A3 Refactor/Simulation/tb3_maze` | Final refactored simulation package `tb3_maze` | Yes, simulation |
| `A3 Refactor/Physical ROS` | Final refactored physical package `mtrx3760_project1` | Yes, TurtleBot |
| `A1 Maze Simulation/tb3_third_parties` | Existing ROBOTIS simulation dependencies | Used by A3 simulation |
| `validation` | Test tools, recorded results and refactor explanation | Report evidence |
| `Report ROS` | Design-diagram sources for the final implementation | Report preparation |

Start with [A3 build and launch instructions](A3%20Refactor/README.md).
The physical controller runs on the PC/laptop; bring up the robot drivers
separately on the TurtleBot. Teleop is a separate tool for manual positioning.

A3 preserves the existing right-wall algorithms and ROS interfaces. It is the
refactored version of A1/A2, with its own clearly labelled source folders.
Historical A1/A2 packages contain `COLCON_IGNORE` to avoid duplicate package
discovery. Do not build the entire repository recursively: it also contains
multiple copies of vendor packages. Use the explicit source paths in the guide.

The final software has regression and Gazebo evidence. Actual autonomous
physical testing of the refactored version is still required before claiming
that it is the demonstrated working code.
