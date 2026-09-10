# Differential-drive ROS 2 simulation

A seven-link differential-drive robot for Ubuntu 24.04, ROS 2 Jazzy, and
Gazebo Harmonic. The repository includes the Fusion CAD source, STL meshes,
plain URDF description, Gazebo world, ROS–Gazebo bridges, RViz configuration,
keyboard teleoperation, and a seeded autonomous-navigation benchmark.

![Differential-drive robot](docs/images/robot_preview.png)

## Architecture

```text
Keyboard teleop → /cmd_vel → ros_gz_bridge → Gazebo physics
Gazebo → odometry, joints, LiDAR, clock → ros_gz_bridge → ROS 2
URDF + joint states → robot_state_publisher → TF tree → RViz

Wheel odometry + IMU → robot_localization → filtered odometry
Saved map + filtered odometry + LiDAR → AMCL + Nav2 → /cmd_vel
```

Gazebo is responsible for physics and sensor simulation. ROS 2 carries
commands, transforms, and sensor data. RViz visualizes the robot, wheel
odometry, TF tree, and laser scan. See [the architecture notes](docs/architecture.md)
for the complete topic and frame layout.

## Repository layout

```text
.
├── cad/
│   ├── Differential Drive Robot.f3d
│   └── cad_geometry_report.json
├── differential_drive_robot_description/
│   ├── meshes/
│   ├── urdf/differential_drive_robot.urdf
│   ├── CMakeLists.txt
│   └── package.xml
├── differential_drive_robot_simulation/
│   ├── config/bridge.yaml
│   ├── config/simulation.rviz
│   ├── launch/sim.launch.py
│   ├── worlds/test_world.sdf
│   ├── CMakeLists.txt
│   └── package.xml
├── differential_drive_robot_navigation/
│   ├── config/
│   ├── launch/navigation.launch.py
│   ├── maps/
│   ├── differential_drive_robot_navigation/
│   └── package.xml
├── differential_drive_robot_dynamic_navigation/
│   ├── config/
│   ├── launch/dynamic_navigation.launch.py
│   ├── maps/ and worlds/
│   ├── differential_drive_robot_dynamic_navigation/
│   └── package.xml
└── docs/
    ├── architecture.md
    ├── verification.md
    ├── images/
    └── reports/
```

The CAD model is the editable design source. The URDF and STL files are a
simulation snapshot and are not updated automatically when the CAD changes.

## Prerequisites

Install Ubuntu 24.04, ROS 2 Jazzy, and the ROS integration for Gazebo
Harmonic. Do not install Gazebo Classic for this project.

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rviz2 \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  python3-colcon-common-extensions \
  python3-rosdep \
  liburdfdom-tools
```

## Workspace setup

Build the repository in its current workspace directory:

```bash
cd ~/Desktop/ROS_Projects/differential-drive-ros2
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths . \
  --ignore-src \
  --rosdistro jazzy \
  -r -y

colcon build --symlink-install \
  --packages-select \
  differential_drive_robot_description \
  differential_drive_robot_simulation \
  differential_drive_robot_navigation \
  differential_drive_robot_dynamic_navigation

source install/setup.bash
```

Only the four robot packages are selected.

## Run the simulation

Start Gazebo, the robot state publisher, ROS–Gazebo bridges, robot spawning,
and RViz:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/ROS_Projects/differential-drive-ros2/install/setup.bash
ros2 launch differential_drive_robot_simulation sim.launch.py
```

The robot always spawns at the fixed pose `x=-5.0`, `y=0.0`, `yaw=0.0` in
both the basic simulation and autonomous-navigation launch files.
Closing Gazebo also shuts down the associated ROS nodes, preventing stale
Nav2 and TF processes from surviving into the next simulation run.
Each launch uses its own Gazebo transport partition and bridges the selected
world's clock. A second basic or navigation launch in the same `ROS_DOMAIN_ID`
is rejected before any simulation nodes start. The launch also checks for
leftover clock/sensor publishers and localization/navigation nodes.

In a second terminal, start keyboard control:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/ROS_Projects/differential-drive-ros2/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The useful differential-drive keys are:

```text
u    i    o      forward-left / forward / forward-right
j    k    l      rotate-left / stop / rotate-right
m    ,    .      reverse-left / reverse / reverse-right
```

Press `k` or Space to send a zero-velocity command before terminating teleop.
The speed controls printed by the teleop program can be used, but Gazebo caps
the robot at 0.5 m/s linear and 1.5 rad/s angular velocity.

## Run autonomous navigation

Launch Nav2 with a reproducible obstacle layout:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/ROS_Projects/differential-drive-ros2/install/setup.bash
ros2 launch differential_drive_robot_navigation navigation.launch.py \
  seed:=42 obstacle_count:=6
```

Manual goal selection is the default. Wait for the terminal message
`Localization initialized at the fixed start pose; waiting for a manual RViz 2D Goal Pose`.
Then, in RViz,
select **2D Goal Pose**, click the destination on the map, and drag to choose
the robot's final heading. The launch always initializes AMCL at the fixed
Gazebo spawn pose, waits for a scan-confirmed pose, and activates Nav2 before
reporting readiness. You can send another goal the same
way after the robot arrives.

To run the original automatic start-to-goal mission, add `auto_goal:=true`:

```bash
ros2 launch differential_drive_robot_navigation navigation.launch.py \
  seed:=42 obstacle_count:=6 auto_goal:=true
```

The automatic destination defaults to `(5.0, 0.0, 0.0)`. Override its map
coordinates and heading from the command line when needed:

```bash
ros2 launch differential_drive_robot_navigation navigation.launch.py \
  seed:=42 auto_goal:=true goal_x:=3.0 goal_y:=-1.5 goal_yaw:=1.57
```

Use `seed:=-1` (the default) for a newly generated seed. The launch validates
that the inflated robot footprint can reach the goal before starting Gazebo.
Generated worlds and layout metadata are written below
`/tmp/differential_drive_navigation/worlds`; trial results are written below
`/tmp/differential_drive_navigation/results`.

Useful launch arguments are `rviz`, `headless`, `auto_goal`, `goal_x`,
`goal_y`, `goal_yaw`, `mission_timeout`, `record_results`,
`generated_world_directory`, and `results_directory`. Mission timeout and
result recording apply only when `auto_goal:=true`.

Stop the launch with **Ctrl+C** and wait for it to finish before restarting.
Do not run `sim.launch.py` alongside `navigation.launch.py`: navigation already
starts Gazebo and the robot. If startup reports existing nodes, stop their
original launch terminal; `ros2 daemon stop` only stops the discovery daemon,
not Gazebo or navigation nodes. A failed essential process shuts down the
stack instead of leaving the remaining nodes running.

For a healthy active run, `ros2 topic info /clock` reports one publisher.
Repeated `TF_OLD_DATA` or backward clock jumps are not normal. A Gazebo world
reset is not a navigation restart: stop and relaunch the full stack to restore
the fixed spawn pose and reinitialize localization together.

## Run dynamic-obstacle navigation

Launch the predictive mode with two static and four moving obstacles:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/ROS_Projects/differential-drive-ros2/install/setup.bash
ros2 launch differential_drive_robot_dynamic_navigation dynamic_navigation.launch.py \
  seed:=42 static_obstacle_count:=2 dynamic_obstacle_count:=4
```

The robot cruises at up to 0.45 m/s and the obstacles patrol at 0.10–0.22 m/s.
Use `robot_speed`, `obstacle_min_speed` and `obstacle_max_speed` launch arguments
to adjust them. LiDAR-derived motion predictions feed the local costmap; global
planning uses observed occupancy and retains valid routes to reduce unnecessary
detours. Collision checks remain active with tighter soft-clearance settings.
Predictive avoidance and an automatic `(5.0, 0.0)` goal are the
defaults. Use `avoidance_mode:=reactive` for scan-only avoidance or
`auto_goal:=false` to select a 2D Goal Pose in RViz.

Stop with Ctrl+C once and wait for the launch terminal to return. The dynamic
launch now stops patrols, shuts down Nav2 through its lifecycle managers, then
stops Gazebo before terminating the remaining processes.

Generated layouts and trial results are stored below
`/tmp/differential_drive_dynamic/`. See
[the dynamic-navigation package guide](differential_drive_robot_dynamic_navigation/README.md)
for all launch arguments, RViz instructions and the read-only runtime health check:

```bash
ros2 run differential_drive_robot_dynamic_navigation check_navigation.py
```

## Verification

The description should convert to seven links, six joints, sixteen mesh URIs,
two model plugins, a GPU LiDAR, an IMU, and a contact sensor. Runtime expectations are about
50 Hz for `/odom`, 10 Hz for `/scan`, and 100 Hz for `/imu/data` (simulation
time). `/joint_states` requests 50 Hz; older Gazebo JointStatePublisher versions
ignore that limit and publish at the physics rate instead. Wall-clock rates are
lower when simulation runs slower than real time.

Use the commands and acceptance checklist in
[docs/verification.md](docs/verification.md).

## Known behavior

`/odom` is wheel odometry. If the robot pushes against an obstacle while the
wheels continue turning, RViz can show motion even though the physical model
is blocked in Gazebo. Autonomous mode fuses this signal with the IMU but cannot
eliminate translational wheel-slip drift. The red obstacle
shape in RViz is the LiDAR scan, not the Gazebo world model.

## CAD and reports

The Fusion archive is stored in `cad/Differential Drive Robot.f3d`. Existing
geometry, mass-property, and validation reports are retained alongside the
repository documentation. CAD source and report data should be revalidated
after changing dimensions, component placement, joint origins, or mesh
exports.

## License

The repository currently remains proprietary; see [LICENSE](LICENSE). Choose
and apply an open-source hardware/software license before inviting reuse or
redistribution. Update the `<license>` and maintainer entries in all four
`package.xml` files if the licensing or ownership information changes.
