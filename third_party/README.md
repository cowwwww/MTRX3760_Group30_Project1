# Simulation dependencies

These are the existing TurtleBot3 Gazebo and description packages moved from
`A1 Maze Simulation/tb3_third_parties/` at commit `0fa5c50`, with their upstream
Apache-2.0 licenses. They include the existing Burger camera model modifications.
They are dependencies, not earlier versions of our controller. The duplicate
vendor trees and unused packages remain available in Git history. Physical
`turtlebot3_bringup` and the LD19 driver come from the robot installation.

Consolidation modification: removed the unused upstream random-drive example
(source/header and CMake target). The project has one driving executable;
simulation model/plugin assets are otherwise retained from the baseline.
