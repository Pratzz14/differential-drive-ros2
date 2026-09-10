# Open a saved map and drive to destinations

Use this workflow after [exploration](exploration.md). It starts the matching
Gazebo environment, loads the saved occupancy map, localizes the robot with
AMCL and uses Nav2 to drive to destinations selected in RViz.

## 1. Close exploration and prepare the terminal

Press **Ctrl+C** in the exploration terminal. The completed exploration session
holds the robot stationary; it is not the saved-map navigation session.

```bash
cd ~/Desktop/ROS_Projects/differential-drive-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

If the workspace has not been built, follow the
[workspace setup instructions](../README.md#workspace-setup) first.

## 2. Choose a saved run

Find the run folder printed at the end of exploration, normally under
`~/ROS_Maps/exploration/`. Open its `summary.md` for the exact reload command.

Alternatively, set the following variable to your chosen folder. **Replace the
example path before running it:**

```bash
saved_run_dir="/absolute/path/to/your/saved/run"

ros2 launch differential_drive_robot_exploration saved_map_navigation.launch.py \
  map:="$saved_run_dir/map.yaml" \
  world:="$saved_run_dir/navigation_arena.sdf"
```

Use the map and world from the **same run**. Keep `map.pgm` beside `map.yaml`.
Older run folders under `/tmp/differential_drive_exploration/results/seed_42/`
also work, but copy the whole folder to permanent storage before temporary
files are cleaned.

## 3. Select a destination in RViz

1. Wait for the map, robot and laser scan to appear and Nav2 to report
   **Managed nodes are active** in the terminal.
2. Select **2D Goal Pose** in the RViz toolbar.
3. Click a destination in clear, mapped free space.
4. Drag to indicate the robot's desired final heading, then release.
5. Watch the planned path and robot movement. After arrival, select another goal.

Avoid obstacles, unknown grey areas and gaps too narrow for the robot. This is
autonomous point-to-point movement, not keyboard teleoperation.

## Localization and troubleshooting

- The launch initializes the expected starting pose automatically. If the robot
  and laser scan do not align with the map, use **2D Pose Estimate** to select the
  robot's actual map location and heading before sending a goal.
- If a previous-session warning appears, stop the previous launch; restarting
  the ROS daemon does not stop its robot nodes.
- If a goal cannot be reached, choose a clear destination in the connected
  mapped area. Partial maps from incomplete exploration support only the routes
  currently represented in that map.
- If map loading fails, check both supplied paths and the companion `map.pgm`.

The saved-map session does not rebuild the map: map server supplies the fixed
map, AMCL estimates position, and Nav2 handles planning and obstacle avoidance.
Saving an occupancy map is different from saving a SLAM pose graph for continued
mapping.

## End the session

Press **Ctrl+C** in the launch terminal. The saved map and report remain on disk
and can be reused in a later session.
