# Verification

Run these checks after building and sourcing the workspace.

## Static checks

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/ROS_Projects/differential-drive-ros2/install/setup.bash

ROBOT_SHARE="$(ros2 pkg prefix --share differential_drive_robot_description)"
SIM_SHARE="$(ros2 pkg prefix --share differential_drive_robot_simulation)"

check_urdf "$ROBOT_SHARE/urdf/differential_drive_robot.urdf"
gz sdf -k "$SIM_SHARE/worlds/test_world.sdf"

export GZ_SIM_RESOURCE_PATH="$(dirname "$ROBOT_SHARE"):$ROBOT_SHARE"
gz sdf -p "$ROBOT_SHARE/urdf/differential_drive_robot.urdf" \
  > /tmp/differential_drive_robot.sdf
```

The converted model should contain:

- Seven links and six joints.
- Sixteen mesh URIs.
- DiffDrive and JointStatePublisher systems.
- One GPU LiDAR and one IMU sensor using `/scan` and `/imu/data`.
- `odom` and `base_link` as the DiffDrive parent and child frames.

The converter can warn that `gz_frame_id` and the noise `type` attribute are
not part of the core SDF schema. They are intentionally preserved extension
fields. Confirm at runtime that `/scan.header.frame_id` is `lidar_link`.

## Runtime checks

With the simulation running:

```bash
ros2 topic hz /odom
ros2 topic hz /joint_states
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom lidar_link
```

Expected results:

- `/odom` and `/joint_states` update near 50 Hz.
- `/scan` updates near 10 Hz using sensor-data QoS.
- `/imu/data` updates near 100 Hz using sensor-data QoS.
- The scan frame is `lidar_link`.
- The TF chain from `odom` to `lidar_link` is available.
- The orange world obstacle appears approximately one metre ahead in the
  initial LiDAR scan.

## Motion acceptance

1. Let the robot settle for 30 seconds and look for sinking, persistent
   rocking, or jitter.
2. Drive forward and reverse.
3. Press `j` and confirm positive angular Z turns the robot left.
4. Press `l` and confirm the robot turns right.
5. Confirm both caster joints move freely.
6. Press `k` or Space and confirm motion stops.
7. Confirm the CAD model and scan remain aligned in RViz during ordinary
   driving.

Wheel odometry can continue to integrate if the drive wheels spin while the
robot is physically blocked. Compare against Gazebo's physical view when
testing collisions.

## Autonomous-navigation checks

```bash
ros2 launch differential_drive_robot_navigation navigation.launch.py \
  seed:=42 obstacle_count:=6 auto_goal:=true

ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /odometry/filtered
ros2 action info /navigate_to_pose
```

Confirm that seed 42 recreates the same six obstacle poses, AMCL publishes
`map → odom`, the EKF publishes `odom → base_link`, Nav2 replans around the
temporary obstacles, and the mission result reports `succeeded`.

For the default manual mode, omit `auto_goal:=true`, wait for localization,
then use RViz **2D Goal Pose** to click and orient a destination. Confirm that
Nav2 accepts each selected goal and that another goal can be selected after
arrival.

## Restart and clock regression checks

Build and run the regression suite:

```bash
cd ~/Desktop/ROS_Projects/differential-drive-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest differential_drive_robot_navigation/test -q
```

Before starting a run, the shared launch guard checks for an existing simulation
in the current ROS domain. While navigation is running, attempting to start
`sim.launch.py` or another `navigation.launch.py` must fail without spawning
another Gazebo instance. There must be exactly one ROS `/clock` publisher.
The bridge takes time from `/world/navigation_arena/clock` (navigation) or
`/world/test_world/clock` (basic simulation) in its private Gazebo partition.
For Gazebo CLI diagnostics, set `GZ_PARTITION` to the value printed in the
launch's `Robot session` message. ROS CLI commands use the usual ROS domain
and do not need this variable.

For manual startup, confirm the readiness message appears, and inspect
`ros2 run tf2_ros tf2_echo map base_link` before sending a goal. The pose should
be close to `(-5, 0, yaw=0)` with no motion commands sent automatically.
Send two successive RViz goals and check that both complete.

Stop Gazebo, or interrupt the launch with Ctrl+C. After the launch has exited,
`ros2 node list` should contain none of this project's nodes. Restart and check
the initial pose again. A lost essential ROS process must also shut down the
stack; successful robot spawning and manual initialization must not.

The installed Jazzy packages can still print one-time upstream diagnostics:

- RViz's first map update can report a GLSL sampler error, described in
  [rviz issue 463](https://github.com/ros2/rviz/issues/463).
- SmacPlanner2D passes a zero possible-collision cost to its shared collision
  checker because it uses radius checking. In this Jazzy implementation the
  checker emits an inflation warning before testing that radius mode; increasing
  inflation does not fix that diagnostic. See
  [SmacPlanner2D](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_smac_planner/src/smac_planner_2d.cpp)
  and [the collision checker](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_smac_planner/src/collision_checker.cpp).

These are distinct from repeated clock jumps, missing map transforms, or a
goal abort. Check actual sensor streams, TF availability and goal results.
