# Verification

Run these checks after building and sourcing the workspace.

## Static checks

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ROBOT_SHARE="$(ros2 pkg prefix --share differential_drive_robot_description)"
SIM_SHARE="$(ros2 pkg prefix --share differential_drive_robot_simulation)"

check_urdf "$ROBOT_SHARE/urdf/differential_drive_robot.urdf"
gz sdf -k "$SIM_SHARE/worlds/test_world.sdf"

export GZ_SIM_RESOURCE_PATH="$(dirname "$ROBOT_SHARE"):$ROBOT_SHARE"
gz sdf -p "$ROBOT_SHARE/urdf/differential_drive_robot.urdf" \
  > /tmp/differential_drive_robot.sdf
```

The converted model should contain:

- Six links and five joints.
- Sixteen mesh URIs.
- DiffDrive and JointStatePublisher systems.
- One GPU LiDAR sensor with `/scan` as its topic.
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
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom lidar_link
```

Expected results:

- `/odom` and `/joint_states` update near 50 Hz.
- `/scan` updates near 10 Hz using sensor-data QoS.
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
