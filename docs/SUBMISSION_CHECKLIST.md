# Project outline and coordinator requirements

Source: Project 1 — Hello Turtlebots, 8 September 2026, pages 2–7; coordinator
clarification supplied by the group on 11 October 2026.

- A1: the shared program must solve a complete maze in simulation, steering from
  sensors. Retain fresh screenshots with simulated LiDAR and camera, and a
  computer-drawn trajectory. Each figure needs a descriptive caption.
- A2: run that wall follower on the robot; the live tutor demonstration is the
  accepted physical functionality evidence. Record platform adaptations honestly.
- A3: submit one refactored, working object-oriented program. Earlier versions
  belong in Git history. Keep classes and ROS responsibilities clearly separated.
- Update report references from the removed A1/A2/A3/experiment paths to
  `src/mtrx3760_project1`. Replace the old simulation class diagram: the final
  simulation now uses the same WallFollowerNode/WallFollower classes as hardware.
  Node diagrams must follow Task 5, with class-labelled node instances and clear
  directed connections. Use names-only UML with ROS relationships; do not draw
  UML relationships between separate executables. Diagnostic graphs do not replace
  design diagrams.
- Refresh the text C++ appendix from the final sources, including authored tests.
  The old appendix no longer represents this checkout. Preserve relevant original
  authorship and acknowledge AI assistance accurately.
- Finish the A0 and A4 responses, cover SIDs and lab section, and AI acknowledgement.
- After committing the final reviewed code, paste this exact text log into the report:

  ```bash
  git log --graph --pretty=format:"%h %ad %an %s" --date=short
  ```

- Create the submission bundle from that same committed repository:

  ```bash
  git bundle create /tmp/project1.bundle --all
  git bundle verify /tmp/project1.bundle
  ```

  Zip the bundle for Canvas. Do not include binaries or build output. A bundle
  excludes uncommitted changes. Do not rewrite the earlier history to remove old
  implementations: that history is explicitly required.

The current report and existing source ZIP are separate artifacts and have not
been rewritten by this repository consolidation.
