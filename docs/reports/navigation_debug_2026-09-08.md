# Navigation debugging and runtime verification — 2026-09-08

Tested locally on ROS 2 Jazzy and Gazebo Harmonic in
`~/Desktop/ROS_Projects/differential-drive-ros2`.

## Corrections

- Stopped identified stale robot launch processes, including an unresponsive
  localization lifecycle manager. No source or user data was deleted.
- Added a shared per-domain launch lock and live ROS graph preflight check.
- Assigned each launch a fresh Gazebo transport partition; selected the world
  explicitly for spawning and bridged its world-specific clock.
- Launched Gazebo directly so shutdown targets the simulator rather than an
  intermediate shell. Essential process exits stop the remaining stack.
- Replaced `bridge_node` with `parameter_bridge`. The former crashed during
  multiple shutdown checks; the latter exited cleanly in the GUI trial.
- Enabled the missing Gazebo IMU system in both worlds.
- Initialized AMCL after clock, scan, filtered odometry and active AMCL were
  available; activated Nav2 only after scan-confirmed localization.
- Preserved manual RViz goals by default and automatic goals through
  `auto_goal:=true`.
- Increased Nav2's action acknowledgement timeout from 20 ms to 1000 ms after
  reproducing a goal abort under simultaneous Gazebo/RViz load.

## Results

| Check | Result |
| --- | --- |
| Build all three packages | Passed |
| Navigation package tests | 16 passed |
| Seed 42, full automatic route, headless | Succeeded in 59.962 s; 11.534 m travelled |
| Seed 42, physical Gazebo goal position | `(4.914, 0.127)`; about 0.154 m from `(5, 0)` |
| Manual mode initial localized position | `(-5.0022, 0.0006)` |
| First `/goal_pose` destination `(-4, 1)` | Succeeded |
| Subsequent `/goal_pose` destination `(-5, 0)` | Succeeded |
| Manual test stream checks | Clock, scan, IMU, filtered odometry and nonzero commands received |
| Manual test clock | One publisher; zero backward jumps over 17,014 received clock messages |
| Concurrent basic launch during navigation | Rejected before spawning processes |
| Essential node exit | Remaining navigation stack shut down |
| Seed 7, full route with Gazebo GUI and RViz | Succeeded in 56.164 s; 11.377 m travelled |
| Seed 7, physical Gazebo goal position | `(4.905, 0.110)`; about 0.146 m from `(5, 0)` |
| Basic simulation physical spawn | `(-5.000, 0.000)`, yaw zero |
| Basic simulation streams | One clock publisher, IMU data and odom-to-base TF confirmed |
| Final cleanup | No project ROS nodes or Gazebo processes remained |

Goal positions were checked in Gazebo's world pose stream, independently of
AMCL. The seed 7 localized goal error was 0.183 m, within the configured 0.20 m
goal tolerance. These trials validate the tested layouts, not every possible
random layout or arbitrary destination.

One-time RViz shader and SmacPlanner2D diagnostics are documented in
[verification.md](../verification.md). They remain upstream messages; the
successful trials contained no recurring `TF_OLD_DATA` or backward clock jumps.
