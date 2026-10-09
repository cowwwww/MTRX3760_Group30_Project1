# Structural assessment and scenario results

This suite tests the previously supplied controller without changing its source. The original controller is also compiled for comparison. Both receive simulated scans and, where supported, odometry; only the environment knows the wall coordinates, scenario name and completion goal.

## Why the partial-loss corner succeeds

The original `WallFollower::UpdateScan` sets `valid_ = front.valid && right.valid && diagonal.valid`. Failure of any sector sets `WaitingForScan`. `Command` then returns a zero-initialised velocity. This loses the manoeuvre context just when a wall edge changes what those sectors observe.

The refactor separates a fresh, broadly usable scan from the measurements needed for each motion. `FollowWall` still requires a usable nearby right wall. A right corner is entered only after several finite returns demonstrate an opening beyond the remembered nearby wall plane. The sequence is `AdvanceRight -> TurnRight -> FindWall -> FollowWall`:

- Advancement uses the measured edge location, target wall offset and laser position; odometry measures progress along the incoming wall heading.
- Pivoting uses angular error to the measured incoming wall heading minus 90 degrees, assuming orthogonal corners. Its completion is measured, not timed. It continues through an unusable right-distance sector when current swept-body clearance is sufficiently observed.
- Acquisition holds the outgoing heading and requires three near-wall scans before ordinary tracking resumes. Missing readings never become fictitious wall distances or evidence of an opening.
- Stale inputs, insufficient clearance, large unobserved spans or violated manoeuvre bounds stop motion with a reason. Timeouts are watchdogs, not a route schedule.

The change is therefore a functional correction to A2, organised inside the existing A3 classes. The normal proportional distance/heading steering equation remains. `Settings`, `ScanProcessor`, `WallFollower`, the ROS adapter, publisher and recorder keep their existing responsibilities; odometry now also reaches the controller and the adapter publishes diagnostics. There is no additional map, global planner or node that supplies maze coordinates.

The project brief requires sensor-driven right-wall following and rejects ignoring sensors to drive a pre-programmed path. It also permits simplifying starting assumptions and asks for refactoring or rewriting into clear object-oriented code. This approach fits those structural requirements for the right-angled mazes under discussion. A 90-degree relative corner assumption is a declared geometry assumption, not a stored sequence of turns. Its applicability to oblique/curved corners is not established. Exact alignment with Weeks 1–6 must be checked against the team's lecture material, which was not supplied. The brief also requires acknowledgement of AI assistance. This suite does not replace A1's Gazebo/camera/LiDAR demonstration or A2's live physical demonstration.

## Dead-end behaviour actually observed

In the 0.6 m corridor with an end wall 2 m down the outgoing branch, the refactor approaches to approximately 0.283 m body clearance when left avoidance starts. The first left pivot turns about 186.7 degrees and releases when forward clearance and the right-wall heading become usable. Steering then corrects the wall offset, with a further small avoidance episode before stable return tracking. The minimum body clearance to the end wall across the whole run is approximately 0.180 m. It returns to the branch entrance at 84.4 s, heading 89.5 degrees from the original starting direction, with a right-wall range of 0.254 m and no body contact.

This is a successful reversal and resumption of right-wall following. It is not a constant-clearance semicircular manoeuvre. At 0.8 m width, the turnaround uses several left-avoidance episodes instead of one clean 180-degree pivot. The test must not be described as a smooth wall-hugging turn throughout. The desired ordinary-tracking offset is 0.25 m from the laser origin, approximately 0.16 m side body clearance with this model, rather than touching the wall.

The left-avoidance branch currently commands constant angular speed until its sensor-based release condition is met, with an upper bound of approximately 188.6 degrees. The 186.7-degree episode is close to that bound. Physical sensing noise and actuator response could therefore change whether it releases or stops at the bound. A successful synthetic return does not settle the quality of this branch.

All eleven dead-end scenarios complete for the refactor: end walls at 1, 2 and 3 m, both widths, persistent partial loss during the first right pivot, temporary fully missing right/front sectors, different first-corner distance, a world rotated by 60 degrees, and a slower synthetic actuator response. These settings are environment variations; the default controller gains and motion settings are unchanged across them.

## Outcomes and limitations

There are 25 environments and 50 runs. The refactor completes 22 goals, holds and then times out with a persistent fully blind right sector, and fails two navigation cases. The original completes 20 goals, stops under four persistent partial-loss cases, and holds under complete right blindness. No body contact is detected in either version's runs.

Two refactor failures remain visible:

1. **Wall-attached protrusion in the 0.6 m corridor:** a 0.30 m long, 0.10 m deep step in the right boundary confuses the measured corner approach. `AdvanceRight` approaches the opposite wall; at 16.3 s it reports `corner_advance_obstructed` with approximately 0.049 m forward clearance, below the 0.05625 m configured braking allowance at search speed. It holds, then latches `manoeuvre_timeout`. The same protrusion case completes in the 0.8 m corridor. The refactor's corner/heading reasoning therefore still needs improvement for stepped boundaries.
2. **Old 0.20 m target / 0.45 m lost-wall threshold in the 0.8 m corridor:** the approach turns too much, reaches about -29 degrees, and loses the usable near-wall measurement before the first corner. It reports `right_wall_unobserved`. This is the earlier parameter-sensitive tracking limitation, not a dropout-recovery result.

The clear synthetic cases show that the structural change is useful, but do not establish general maze robustness. The dead-end pivots and these two failures deserve physical investigation before claiming final functionality.

## Source and measurement provenance

`original/` is the portable core from baseline Git commit `5499600b51fbc0a720641ff3dad56b02d778594e`. `new/` is byte-for-byte the portable core from `turtlebot_a2_refactor_20261008`. Hashes are in `source_manifest.json`. No later experimental heading changes are used.

The raycast model uses 503 rays, 10 Hz scans, 20 Hz control, deterministic range variation up to 4 mm, the recorded -0.032 m laser offset, a conservative rectangular body model and a first-order actuator response. Odometry is ideal. Materials, LiDAR firmware effects and wheel slip are not modelled. All paths and figures derive from controller-driven CSV logs. The added world rotation and corner-distance variations show that the controller is not receiving the original maze's absolute waypoints.

Partial-loss masks remove the same ten narrow-sector ray positions as the recorded Floor 01 snapshot, plus ten outer-sector rays, leaving 4/14 and 22/42 usable. Distances remain geometric ray hits. The episode begins during the first clockwise pivot and ends once its angle reaches 85 degrees; it cannot reactivate on the later dead-end reversal. Temporary faults last 0.8 s in the environment. These fault timings are not movement instructions supplied to the controller.
