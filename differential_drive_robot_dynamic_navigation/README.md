# Dynamic-obstacle navigation

This package adds a seeded Gazebo arena with static and moving rectangular obstacles. Moving obstacles patrol between generated endpoints at 0.10–0.22 m/s; robot cruise speed defaults to 0.45 m/s. The robot detects motion from LiDAR clusters, estimates velocity, predicts occupied cells, and feeds those predictions into a Nav2 costmap layer.

## Build and run

```bash
cd /home/pratik/Desktop/ROS_Projects/differential-drive-ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to differential_drive_robot_dynamic_navigation
source install/setup.bash
ros2 launch differential_drive_robot_dynamic_navigation dynamic_navigation.launch.py seed:=42
```

Useful launch arguments:

```bash
ros2 launch differential_drive_robot_dynamic_navigation dynamic_navigation.launch.py \
  seed:=42 static_obstacle_count:=2 dynamic_obstacle_count:=4
ros2 launch differential_drive_robot_dynamic_navigation dynamic_navigation.launch.py \
  avoidance_mode:=reactive
ros2 launch differential_drive_robot_dynamic_navigation dynamic_navigation.launch.py \
  headless:=true rviz:=false auto_goal:=true mission_timeout:=180
```

`predictive` is the default. The `reactive` mode disables the LiDAR motion tracker and leaves Nav2's normal obstacle layer to react to the latest scan. Each run writes its generated world/layout and a trial result JSON under `/tmp/differential_drive_dynamic/` unless another directory is supplied.

The controller uses Gazebo pose feedback only to drive the generated patrols. Navigation decisions use ROS LiDAR data, so the prediction path remains sensor-based.

## Speed settings

The faster demonstration settings are the defaults. You can set them explicitly:

```bash
ros2 launch differential_drive_robot_dynamic_navigation dynamic_navigation.launch.py \
  seed:=42 robot_speed:=0.45 obstacle_min_speed:=0.10 obstacle_max_speed:=0.22
```

For the earlier, slower speeds, use `robot_speed:=0.30 obstacle_min_speed:=0.05
obstacle_max_speed:=0.15`. Speed arguments change speeds, not the seeded patrol
geometry. Robot cruise speed is limited to 0.10–0.50 m/s; box speeds must be
positive, ordered and no greater than 60% of the selected robot cruise speed.
This ratio is a simulation configuration bound, not a collision-safety guarantee.
The robot still slows or stops near hazards. The faster boxes brake at endpoints
and use pose feedback to correct displacement and heading.

## RViz and healthy startup

RViz opens in `map`, with the robot model, red LiDAR returns, local costmap,
global plan and cyan track arrows enabled. The global costmap and TF axes are
available but off by default to keep the view readable. Map and robot-description
subscriptions use transient-local durability so they work even if RViz opens late;
the scan subscription uses sensor-compatible best-effort reliability.

Nav2 lifecycle activation is delayed five seconds and RViz opens after ten
seconds to reduce concurrent startup work. This launch defaults Fast DDS to
asynchronous publication, so DDS writes do not block the publishing executor
thread; an explicitly set `RMW_FASTRTPS_PUBLICATION_MODE` is preserved. This
setting is local to the launched processes, not a system configuration change.
See [the ROS Fast DDS publication-mode documentation](https://github.com/ros2/rmw_fastrtps/blob/jazzy/README.md#change-publication-mode).

The launch initializes localization at the robot's spawn pose. Allow startup to
finish: the mission waits for active Nav2, a fresh valid laser scan, and its map
transform before starting patrols or sending a goal. For manual navigation:

```bash
ros2 launch differential_drive_robot_dynamic_navigation dynamic_navigation.launch.py \
  seed:=42 auto_goal:=false
```

Wait for `Ready for an RViz 2D Goal Pose`, then use **2D Goal Pose**. You do not
normally need **2D Pose Estimate** unless you manually relocate the robot.

In a second terminal, check the running stack without moving anything:

```bash
cd /home/pratik/Desktop/ROS_Projects/differential-drive-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run differential_drive_robot_dynamic_navigation check_navigation.py
```

The check runs for 15 seconds and exits nonzero if required messages, robot/laser
transforms or clock health are missing. Expected: `healthy: true`, one clock
publisher, zero backward jumps, and approximately 10 Hz LiDAR **in simulation
time**. Wall-clock rates can be lower when Gazebo runs slower than real time.

Stop the entire launch with Ctrl+C before restarting. Do not use Gazebo's reset
time/world button while Nav2 is running: it invalidates localization and TF
history. The launch rejects a second stack in the same ROS domain and isolates
Gazebo transport to prevent competing clocks. If it reports existing publishers,
stop their owning launch terminals; restarting the ROS daemon does not stop them.

## Prediction stability and tests

Predictions use timestamped odometry transforms, reject large wall clusters and
clusters near occupied cells in the known static map (including partial walls),
fit velocities over multiple scans, and expire lost tracks and RViz markers.
The prediction layer marks world coordinates in the current master costmap,
snapshots data per update, validates frames, and clears expired/reset prediction
bounds without erasing the normal obstacle layer. A three-second controller
failure tolerance allows the robot to wait stopped for a temporary obstruction;
collision detection and the collision monitor remain enabled.

The position-only global planner uses a circular robot footprint that contains
the chassis at every heading. Local collision checks retain the physical polygon
footprint. The package behavior tree treats Jazzy's transient
`START_OCCUPIED` planning error as recoverable, with at most six recovery retries,
instead of immediately abandoning the goal. It does not retry forever or bypass
collision checks. Mission logs show remaining distance/recovery counts every
five seconds, and explicitly report when an aborted goal is no longer active.

## Closer, more consistent routes

Global planning now prefers more direct routes: inflation is 0.50 m (previously
0.65 m), its cost decays faster, and Smac's cost-travel multiplier is 2.0 instead
of 5.0. This reduces the **soft clearance preference**, not the robot's physical
footprint or collision protection. The pursuit controller's cost scaling matches
the local inflation layer so it can still slow near obstacles.

The global costmap uses the static map and current LiDAR occupancy. One-second
motion predictions remain in the **local** costmap; projecting speculative trails
into the global map no longer keeps changing which side of the arena looks best.
The behavior tree checks its route at 2 Hz, keeps a valid path, refreshes it after
10 simulation seconds, and replans for a new goal. If a route becomes invalid,
it allows a one-second wait/recheck before replanning. Persistent blockage still
triggers a new path and bounded recovery. Wait actions increment Nav2's recovery
counter even when no costmap-clearing or backup recovery was needed.

Short safety stops are still expected when a box crosses immediately in front
of the robot. This is predictive costmap avoidance with regulated pure pursuit,
not a time-optimal trajectory planner or a guarantee of uninterrupted movement.

## Orderly shutdown

Press **Ctrl+C once in the launch terminal**, then wait for it to return to the
shell before restarting. Patrols are stopped and active navigation goals are
canceled first; navigation and localization are shut down through their lifecycle
managers while simulation time and bridges
are still running. Gazebo then receives a stop request in this session's isolated
transport partition, and launch terminates/reaps the remaining processes.

On Linux, child processes are isolated from the terminal's initial Ctrl+C so it
cannot interrupt them halfway through the lifecycle teardown. Service calls have
bounded waits; failures are logged and fall back to process signals. Children
get up to 15 seconds after SIGINT before SIGTERM escalation. A routine shutdown
usually takes several seconds; an unavailable service can take longer. Closing
the Gazebo window also ends the stack. Essential runtime crashes still stop the
session and remain visible as errors; they are not hidden by this change.

```bash
colcon test --packages-select differential_drive_robot_dynamic_navigation differential_drive_robot_navigation
colcon test-result --verbose
```

These changes do not guarantee success for every random crowding pattern. Keep a
seed when reporting a failure, together with the launch log, health-check output,
and trial JSON. A successful action result is recorded only when Nav2 reports
success; startup failures now terminate with an error instead of appearing ready.

See [VERIFICATION.md](VERIFICATION.md) for the tested routes, measurements and
remaining non-blocking startup diagnostics on the local graphics/simulation stack.
