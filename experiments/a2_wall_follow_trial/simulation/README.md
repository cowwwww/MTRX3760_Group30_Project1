# TurtleBot varied-maze software scenario suite

Run the actual original and previously supplied refactor against 25 environments. The refactor source is unchanged. Read `ASSESSMENT.md` for the structural explanation, project alignment, measured dead-end behaviour and failures.

Requires CMake, a C++14 compiler and Python 3. From this extracted directory:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
python3 run_suite.py --build build --output rerun-results
```

The last command records both versions in each environment and writes CSVs, `results.json` and `summary.csv`. It reports actual goal outcomes, including failures. No ROS or robot connection is used.

To run a right turn into a dead end 2 m down the outgoing corridor:

```bash
./build/new_runner --width 0.6 --dead-end true --depth 2 --output dead-end.csv
```

For the same layout with artificially missing right returns during the first right turn:

```bash
./build/new_runner --width 0.6 --dead-end true --depth 2 --fault partial --output dead-end-dropout.csv
```

To change the environment, use these option/value pairs:

| Option | Meaning |
|---|---|
| `--width 0.8` | Wider incoming and outgoing corridors |
| `--edge 2` | First right edge 2 m from the starting position |
| `--depth 3` | Dead-end cap 3 m from the outgoing inner wall's start |
| `--junction true` | A straight route also exists; the right branch is optional |
| `--rotation 60` | Rotate the entire physical scene and robot start by 60 degrees |
| `--start-yaw 8` | Start 8 degrees skewed relative to the incoming corridor |
| `--start-y 0.05` | Start 0.05 m left of the centreline |
| `--lag 0.20` | Slower synthetic velocity response |
| `--obstacle true` | Add a 0.30 m long / 0.10 m deep step attached to the right wall |
| `--fault transient_right` | Remove the whole right sector for 0.8 s during the first pivot |
| `--fault transient_front` | Remove front-sector rays for 0.8 s after entering the outgoing corridor |
| `--fault blind` | Keep the right sector fully missing; safe stopping is expected |
| `--max-time 180` | Software-test observation horizon; not a controller movement duration |

Replace `new_runner` with `original_runner` to run the original against identical inputs and default parameters. `scenarios.json` defines the complete matrix. Controller defaults here are wall range 0.25 m, lost wall 0.65 m, forward/search speeds 0.08/0.05 m/s, front stop/release 0.28/0.38 m and turn limit 0.60 rad/s. Target wall distance is from the LiDAR origin, not body clearance. The explicitly labelled archived-settings scenario changes only the target and lost-wall thresholds.

For an open outgoing corridor, completion requires a right turn, return to `FollowWall`, and at least 0.30 m of travel beyond the outgoing inner wall's start. For a dead end, the robot must additionally approach the cap within 0.40 m body clearance, reverse its heading and return to near the branch entrance in `FollowWall`. A short pivot alone cannot count as dead-end success. All runs stop immediately if the independent body/wall contact checker detects contact.

The CSV contains 20 Hz pose, commands, state/reason, fault activity, usable right-sector counts, radial front-sector distance, forward body clearance, heading/pivot validity, right-wall distance, end-cap body clearance and goal flags. The replay uses these computed poses and controller states; it does not script the robot's route. Coordinates and yaw in the CSV are expressed in the scenario's local frame even for a rotated world.

`results/` contains the actual logs from this run. All 11 dead-end cases complete for the refactor. Across all 25 cases it completes 22, safely holds for one permanent sensor failure and has two navigation failures. Its dead-end reversal is not a smooth constant-clearance wall-hugging arc. See `ASSESSMENT.md` before interpreting the counts as robustness evidence.

These are synthetic tests with ideal odometry. They do not replace the project's complete Gazebo demonstration or its live robot demonstration. Use the physical build, recording and test instructions in the existing refactor package when ready to validate on the university machines.
