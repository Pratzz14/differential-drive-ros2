# Run autonomous exploration and mapping

Use this workflow to start the custom robot without a map, explore the generated
Gazebo arena, and save a map that can later be used for navigation.

## 1. Prepare the workspace

Install dependencies and build once using the [workspace setup instructions](../README.md#workspace-setup).
In each new terminal, run:

```bash
cd ~/Desktop/ROS_Projects/differential-drive-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Close any previous robot launch with **Ctrl+C** before starting another session.

## 2. Start exploration

```bash
ros2 launch differential_drive_robot_exploration exploration.launch.py \
  seed:=42 obstacle_count:=6
```

Gazebo and RViz open automatically. SLAM Toolbox builds the map while the
explorer chooses frontier destinations and Nav2 drives the robot. Do not start
a separate SLAM or navigation launch alongside it.

In RViz, watch the map expand. Yellow/orange points mark candidate exploration
destinations at the boundary between known and unknown space. The robot prepares
new goals while moving, but may still stop for turns or obstacle avoidance.

## 3. Review the result

The run succeeds when coverage is at least 98% and no reachable frontier remains
after the completion checks. Reaching the coverage threshold alone does not end
the run. If bounded recovery cannot make progress below target, the result is
marked **incomplete** and the partial map is still saved.

At the end, the robot stops and RViz keeps the final map and metrics visible.
The terminal prints the result and the exact output folder. Each run has a new
folder under `~/ROS_Maps/exploration/` containing:

- `summary.md`: readable metrics and a ready-to-copy saved-map navigation command.
- `result.json`: machine-readable metrics and completion status.
- `map.yaml` and `map.pgm`: map metadata and occupancy image; keep them together.
- `navigation_arena.sdf` and `layout.json`: the matching environment.

Metrics include reachable-area coverage, simulated exploration time, distance
travelled and a simulation accuracy score. Coverage and accuracy use Gazebo
ground truth; a high accuracy score does not imply that every area was mapped.

Press **Ctrl+C** after reviewing the results. Hold mode intentionally blocks
movement; to drive to destinations, start the separate
[saved-map navigation workflow](saved-map-navigation.md).

## Optional run modes

Add these arguments to the exploration command when needed:

- `finish_behavior:=shutdown`: save the result and close automatically.
- `mission_timeout:=600.0`: bound the run to 600 simulated seconds; the default is no mission timeout.
- `headless:=true rviz:=false`: run without the Gazebo or RViz windows.
- `output_directory:=/absolute/path/to/results`: change the parent output directory.
- `stop_on_incomplete:=false`: explicitly keep retrying instead of ending an incomplete run.

For repeated trials with aggregate CSV/JSON metrics:

```bash
ros2 run differential_drive_robot_exploration run_exploration_benchmark \
  --seeds 7 21 42 --headless
```

Batch trials close automatically and use a 900-second simulated mission timeout
by default. See [architecture](architecture.md#autonomous-exploration-data-flow)
for the internal data flow.
