# Stability verification — 10 September 2026

## Pre-commit verification

Rebuilt the dynamic-navigation package, existing navigation package and their
workspace dependencies from the repository root.
The build passed. The fresh regression run reported **63 checks, zero errors,
failures or skips**: 35 dynamic Python tests, seven C++ layer tests, 19 existing
navigation tests and two CTest wrapper results.

A fresh seed-42 Gazebo GUI + RViz run reached active navigation. The installed
health checker reported healthy robot/laser/wheel transforms, 10 Hz LiDAR in
simulation time, one clock publisher and zero backward clock jumps. Ctrl+C
during navigation canceled the active goal, completed both lifecycle-manager
shutdown requests and stopped Gazebo. Every launched process exited cleanly;
there was no signal escalation or crash. Expected cancellation was logged at
INFO level. This was a startup/movement/shutdown smoke test; the complete-route
trials are recorded below.

The user's later multicast `No such device` messages did not recur in this
smoke test, but the underlying interface problem has not been reproduced or
fixed. The proposed `GZ_IP=127.0.0.1` workaround was not saved into the launch
and is not verified by this commit. Existing RViz shader and Smac configuration
diagnostics remain visible.

Evidence: `/tmp/dynamic-precommit-smoke.log`,
`/tmp/dynamic-precommit-health.json`, and the workspace's colcon test results.

## Follow-up: direct routes and ordered shutdown (current settings)

The earlier 0.65 m inflation and cost multiplier 5.0 biased paths away from
obstacles. Replanning every second, including speculative motion in the global
costmap, also encouraged route changes for short-lived crossings.

Current changes:

- Inflation radius 0.50 m, cost scaling 5.0, Smac cost multiplier 2.0. The hard
  global radius/padding and local polygon/padding remain unchanged. Pursuit cost
  scaling is explicitly matched to local inflation; collision checking and the
  safety monitor remain enabled.
- Global planning uses observed occupancy; one-second predictions remain local.
  The behavior tree validates at 2 Hz, retains valid routes for up to 10 seconds,
  and handles new goals or persistent invalidity. Invalid routes get a bounded
  one-second wait/recheck before a new plan.
- The user's 11:06 launch log showed collision-monitor exit -11 after SIGTERM
  escalation and Gazebo failing its five-second SIGINT timeout. These were real
  shutdown failures, not just cosmetic Python tracebacks.
- The dynamic launch isolates terminal signals until orderly teardown completes:
  stop patrols, cancel active navigation, shut down navigation/localization via
  lifecycle services, stop this Gazebo server via its partition-scoped service,
  then signal/reap children. Gazebo server and GUI run separately. Service waits
  are bounded, and failure falls back to launch's normal signal escalation.

Route trials (default speeds, predictive mode):

| Seed / boxes | Mode | Result / wall time | Published global plans | Minimum robot-centre distance to divider |
| --- | --- | --- | --- | --- |
| 42 / 2 static + 4 moving | Gazebo GUI + RViz | Succeeded / 69.069 s | Not independently recorded | Not recorded |
| 7 / 2 static + 4 moving | Gazebo GUI + RViz | Succeeded / 84.304 s | 4 | 0.394 m |
| 42 / 4 static + 6 moving | Headless | Succeeded / 66.162 s | 5 | 0.418 m |

All recorded plans crossing the divider in the latter two trials stayed on its
positive-y side: no whole-arena side switch. For comparison, the earlier seed-7
trial below measured 0.637 m minimum centre-to-divider distance. These distances
are **not chassis-edge clearance**, and timings vary with graphics/CPU load.
Both instrumented trials reported healthy TF, one clock, no backwards jumps,
10 Hz simulated LiDAR, moving patrols and nonempty predictions. The dense trial
reported no chassis contacts; its six maximum box commands were 0.197, 0.128,
0.219, 0.205, 0.109 and 0.154 m/s. Robot odometry peaked at 0.452 m/s.

The GUI seed-7 and dense trials reported one and three wait actions respectively.
Nav2 counts these in its recovery feedback; neither required costmap-clear,
backup or spin recovery. Safety stops for crossings still occurred and remain
intentional. This is not a promise of continuous motion in every crowded layout.

Ctrl+C tests after a completed GUI mission, during a GUI return goal, and after
the dense headless mission all exited cleanly, with no SIGTERM/SIGKILL escalation
or process crash. Navigation/localization teardown and Gazebo stop requests
completed. The first trial exposed a first-service discovery timeout; increasing
bounded discovery from one to three seconds resolved it in subsequent runs.
Explicit goal cancellation was then added before lifecycle deactivation to avoid
aborting an otherwise active goal during shutdown.

Evidence: `/tmp/dynamic-direct42.log`, `/tmp/dynamic-direct7.log`,
`/tmp/dynamic-direct7-probe.json`, `/tmp/dynamic-direct7-return.log`,
`/tmp/dynamic-direct-dense.log`, `/tmp/dynamic-direct-dense-probe.json`,
`/tmp/dynamic-direct-final-build.log`. The existing RViz shader and Smac2D
configuration diagnostics remain visible; they did not prevent these runs.

## Follow-up: divider stall and faster demonstration

Historical settings and results, superseded by the direct-route tuning above.

The user's subsequent seed-42 run did fail: the planner reported `Start occupied`
at (-0.19, 2.21), then the navigator aborted after 52.261 seconds. This was not
simply a paused simulator or a goal that was still running. The initial successful
trials below did not cover this failure reliably.

Follow-up changes:

- Global planning now uses a 0.34 m circular robot radius plus 0.02 m padding,
  rather than the narrow inscribed radius of the directional chassis polygon.
  Local collision checking keeps the physical polygon with 0.03 m padding.
  Inflation extends 0.65 m; path cost preference and smoothing retain clearance.
- The LiDAR tracker also rejects clusters near occupied static-map cells,
  including small, partially visible wall fragments whose centroids can shift.
- A bounded behavior tree handles Jazzy's `START_OCCUPIED` error through recovery
  rather than immediate goal abandonment. Invalid-goal errors are not added to
  this retry condition. Recovery has a maximum of six retries.
- Robot cruise speed defaults to **0.45 m/s**, boxes to **0.10–0.22 m/s**. Speed
  arguments are validated together; box feedback brakes near endpoints and
  compensates for displacement/heading. Collision detection remains enabled.
- Nav2 lifecycle activation and RViz startup are staggered. After intermittent
  service startup timeouts were observed, launch-local Fast DDS publication was
  changed to asynchronous (explicit environment overrides are preserved).
- Mission progress is logged every five seconds. Aborted/timed-out goals are
  explicitly reported as inactive, not silently left looking like active goals.

Final full-GUI trials using the resumed build:

| Seed | Action result | Wall time | Minimum centre-to-divider clearance | Maximum measured robot speed |
| --- | --- | --- | --- | --- |
| 42 | Succeeded, zero recoveries | 120.254 s | 0.633 m | 0.453 m/s |
| 7 | Succeeded, zero recoveries | 118.811 s | 0.637 m | 0.452 m/s |

Both runs had healthy map/robot/laser transforms, 10 Hz scans in simulation time,
one clock publisher and zero backward clock jumps. Nonempty prediction clouds
and patrol motion were observed. Maximum box command in seed 42 was 0.219 m/s.
An earlier headless seed-42 follow-up trial also succeeded in 56.202 wall seconds.
The full GUI reduces real-time factor on this machine: higher m/s settings do
not guarantee a shorter wall-clock trial when rendering load changes.

Regression results: **52 reported checks, zero errors, failures or skips**
(24 dynamic Python tests, seven C++ layer tests, 19 existing navigation tests and
two CTest wrapper results). Added cases cover static wall fragments, unknown map
cells, invalid speed ranges, seeded geometry preservation, rotated patrol
commands, footprint containment and the bounded `START_OCCUPIED` condition.

Evidence: `/tmp/dynamic-speed42-gui2.log`, `/tmp/dynamic-speed42-gui2-probe.log`,
`/tmp/dynamic-speed7-gui-retry.log`, `/tmp/dynamic-speed7-gui-probe.log`,
`/tmp/dynamic-speed-final2-tests.log`, `/tmp/dynamic-speed-rviz.png`.

The installed graphics stack still logs its one-time shader diagnostic, and
Smac can log a collision-check optimization warning during configuration. These
did not prevent either final run from completing. This is representative route
testing, not a guarantee of every random encounter or a safety certification.

## Initial verification (historical)

Workspace: repository root.
ROS 2 Jazzy and the locally installed Gazebo Harmonic stack were used.

## Reproduced failure and fixes

The original RViz preset omitted RobotModel and used scalar topic settings rather
than ROS 2 topic/QoS properties. The baseline seed-7 navigation trial aborted with
`Start occupied` after repeated collision-recovery attempts.

The corrected preset includes a late-join-compatible robot description and map,
best-effort LiDAR, readable costmap overlays and the correct MarkerArray topic.
RViz was visually checked: the map, robot meshes and red laser returns render.
The prediction costmap now uses the current master-grid origin and clears old
prediction bounds. The tracker uses nonblocking timestamped TF lookup, a velocity
observation window, wall/implausible-speed filtering, and track/marker expiry.
Startup now requires live scans and localization TF before starting the mission.

## Results

- Final `colcon build --symlink-install --packages-up-to
  differential_drive_robot_dynamic_navigation`: passed.
- Final `colcon test-result --verbose`: **42 checks, zero errors, failures or
  skipped checks**. This includes 14 dynamic Python tests, seven C++ prediction
  layer tests, 19 existing navigation tests, and two CTest wrapper results.
- Seed 7, automatic goal from (-5, 0) to (5, 0): **succeeded**, 67.231 wall seconds
  after the initial prediction/RViz fixes. This run used the original controller
  failure tolerance; later runs also include the three-second stopped tolerance.
- Seed 42, automatic goal: **succeeded**, 103.511 wall seconds.
- Seed 42, return goal sent through `/goal_pose` (the RViz tool's input):
  **SUCCEEDED (action status 4)**; final localized position (-4.834, 0.066).
- During the return trial: 616 laser scans at **10 Hz in simulation time**, one
  clock publisher, **zero backward jumps**, valid robot/laser/wheel map transforms.
- All four patrols moved: observed travel extents were 1.192 m horizontally and
  1.828 m, 0.897 m, 1.036 m vertically, respectively.
- A separate prediction check received 46 nonempty prediction clouds out of 46,
  up to 256 points, and confirmed RViz subscribed to `/dynamic_obstacles/tracks`.
- A second simultaneous launch was rejected before starting another simulation.
- A normal, non-headless Gazebo + RViz launch reached `Ready for an RViz 2D Goal
  Pose`; the installed `check_navigation.py` command returned **healthy: true**,
  with 10 Hz LiDAR, one clock publisher, zero backward jumps and all robot/laser/
  wheel map transforms present. Its executable permission was corrected during
  the installed-command check.

## Scope and remaining caveats

These are successful representative runs, not a guarantee that every random
layout or obstacle encounter can be completed. The contact stream reported no
chassis contacts in the return trial; this is not independent collision-safety
certification or coverage of every robot link.

The local RViz/OpenGL stack emits a one-time indexed-map shader-link diagnostic
at startup even though the map subsequently renders correctly. Initial TF-cache
messages, the KDL root-inertia warning, and occasional scheduler-rate warnings
can also appear during startup. They are not the repeated backward-clock failure
seen earlier. These upstream diagnostics were not hidden or disabled.

Older local Gazebo JointStatePublisher builds publish at the physics rate despite
the requested 50 Hz limit. This can reduce real-time factor on this machine;
sensor rates above are measured in simulation time. No system packages or GPU
drivers were replaced as part of this fix.

Do not reset Gazebo time while Nav2 is active. Stop the launch with Ctrl+C and
restart the whole stack. Use the package README's health check when reporting a
new problem, and retain the seed and logs.

Raw logs from this verification are in `/tmp/dynamic-baseline.log`,
`/tmp/dynamic-fixed-seed7.log`, `/tmp/dynamic-fixed-seed42.log`,
`/tmp/dynamic-manual-trial.log`, `/tmp/dynamic-final-tests.log`,
`/tmp/dynamic-desktop-final.log`, and `/tmp/dynamic-desktop-health.json`.
The RViz rendering capture is `/tmp/dynamic-rviz-final.png`.
