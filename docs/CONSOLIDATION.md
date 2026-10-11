# Consolidation — 11 October 2026

## Requirement and decision

The coordinator requires earlier implementations to remain in Git history and
one program in the final commit. The project outline (A2, page 4) asks for the
wall follower on the robot; A3 (page 5) requires refactored working code and a
history showing its development.

Baseline: `0fa5c50116a754633850638ad6d9d05a35a9ac89` (the merge already pulled
from main). The canonical controller is the physical program formerly at
`experiments/a2_wall_follow_trial/mtrx3760_wall_follow_trial`, now at
`src/mtrx3760_project1`. Its portable control sources and headers remain byte
identical to the baseline. ROS adapter changes rename the node and diagnostic
identifier; the package name and configuration selector were renamed together.
The unused ROS 1 adapter was removed: this submission targets ROS 2 Jazzy.

Both hardware and simulation now launch the same executable, core and physical
settings. Simulation changes the clock setting and adds Gazebo/bridges/RViz;
it does not select a separate steering implementation. The former waypoint
simulation controller and duplicate experiment cores are available in history.
The maze worlds and one copy each of the Gazebo/description dependencies were
retained; upstream licenses were preserved. The unused upstream random-drive
example was removed from the Gazebo dependency to avoid a second driving executable.

The earlier `A1 Maze Simulation/`, `A2 ROS/`, `A3 Refactor/`, `experiments/` and
duplicate `turtlebot3/` trees were removed from the working tree. Obsolete
comparison scripts and layout instructions were removed too. Historical
validation evidence is labelled below, and current run tools use the final
package. Diagram sources now refer to the shared class design and trajectory.

## Validation

Fresh final build location: `/tmp/group30_final` (outside Git). The completed
maze run uses `/tmp/group30_consolidated`, built from the same controller and
model sources; the final build additionally omits the unused vendor example.

- ROS 2 Jazzy build: all three packages built successfully.
- Package discovery: one authored controller package, plus two third-party
  simulation dependencies; no duplicate package names or hidden old controllers.
- Controller/trajectory tests: 29 of 30 passed. The existing
  `controller_corner_dropouts` test fails with persistent missing right/diagonal
  rays on a skewed 0.8 m corner approach. This reproduces the pre-consolidation
  failure; the test and steering code have not been weakened or altered.
- ROS adapter integration: passed, including all seven internal cases. Initial
  sandbox run could not create local DDS sockets; the run with local socket
  access passed. Overall, 30 of 31 CTest entries pass.
- Maze exit detector: all four tests passed.
- Both launch descriptions load and expose their expected arguments. A fresh
  combined simulation launch also started the controller automatically and
  produced sensor-driven wall-following commands.
- Fresh Gazebo default maze: **EXIT_REACHED**, verified east portal crossing,
  at 433.866 simulation seconds; approximately 24.36 m travelled. LiDAR and
  camera streams were active. The final world position was (2.6044, 1.0844) m.
  Evidence is in `validation/consolidation_evidence/2026-10-11/`, including
  camera frames, trajectory CSV, plotted route, metrics and the CTest log.
  No contact sensor was recorded, so this is not a collision-free claim.

A new physical demonstration was not performed. The group previously reported a
successful tutor demonstration of the chosen physical controller. Source identity
supports continuity of the core; it does not replace the tutor's live assessment.
The unresolved dropout case is a known robustness limitation, not a passing test.

## Historical evidence and submission impact

`validation/evidence/2026-10-07`, `validation/refactor_evidence/2026-10-07`,
`validation/layout_evidence/2026-10-07` and `OFFLINE_TESTS_2026-10-07.md` describe
older revisions and contain their original paths/commands. They are retained as
historical evidence only. Do not use their old simulation result as proof that
this consolidated program clears the maze.

The report's old class diagrams, simulation description, source appendix, paths,
Git log and bundled repository need updating after the final code is committed.
See `SUBMISSION_CHECKLIST.md`. This consolidation does not claim those separate
report artifacts are already updated, and no commit or push has been made.

To inspect an earlier file without restoring old folders alongside the final code:

```bash
git show '0fa5c50:A3 Refactor/Simulation/tb3_maze/src/turtlebot3_drive.cpp'
git log --all --oneline -- 'A2 ROS'
```
