# A3 folder clarification — 7 October 2026

The first A3 refactor was committed in the existing A1/A2 package folders. That
made the assignment-stage names confusing for the team. This follow-up gives
the final implementations their own `A3 Refactor` folder and preserves corrected
A1/A2 snapshots for comparison.

## Versions preserved

- Original A1/A2 source, tests and configuration are restored from `ed92004`,
  immediately before the A3 structural refactor. This retains A1's right-wall
  correction, A2's Jazzy linking fix and the offline regression tests.
- Final refactored code comes from `a76e65a`, under
  `A3 Refactor/Physical ROS` and `A3 Refactor/Simulation/tb3_maze`.
- Original folders have historical-use documentation and `COLCON_IGNORE`
  markers. The final package names and launch commands remain unchanged.
- The root README identifies the final demo packages. A3 build commands use
  explicit source paths; vendor dependencies stay in their existing locations.
- Final simulation convenience scripts use `~/tb3_a3_build` and locate the
  existing vendor packages from the repository root. Class-diagram filenames
  now explicitly name A3 physical/simulation designs.
- This is a new commit after the original refactor; no Git history is rewritten.

## Verification

Tested layout/runtime revision: `3103d5aa82ed3938147d3de1afd0d626435d825d`.
Evidence is saved in `layout_evidence/2026-10-07`.

| Check | Result |
|---|---|
| Historical package source/build/configuration identity | 25 files match `ed92004`; documentation and new ignore markers deliberately differ |
| Final source/test/configuration identity | 39 files match the packages at `a76e65a`; final simulation CMake differs only in a dependency-location comment |
| Package discovery across A3 and historical A1/A2 packages | Finds only `mtrx3760_project1` and `tb3_maze`, both under A3 |
| Fresh physical colcon build and install | Passed |
| Fresh simulation/dependency colcon build and install | Three packages passed |
| Physical regression suite | 33/33 CTest entries passed, including the five-scenario ROS integration entry |
| Simulation regression suite | Eight GoogleTest cases passed in one CTest entry |
| Installed executable and launch-argument discovery | Passed for both packages |
| A1 and A2 exact before/after comparisons | 2,000 sequential inputs each match `ed92004` exactly, using the final A3 paths |
| Simulation helper shell syntax | Passed |

No new Gazebo maze run or physical robot trial was performed for this folder
change. The existing refactor maze evidence remains tied to its recorded
revision. Identical controller code and the checks above support the relocation;
actual autonomous physical testing of the final version remains required.

For deployment, follow `../A3 Refactor/README.md` and copy the complete **A3
physical package**, then rebuild the PC/laptop workspace. A Git pull alone does
not update an installed controller. Record the source revision actually built
when collecting the final physical maze results.
