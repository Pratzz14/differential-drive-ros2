# System architecture

## Responsibilities

- Gazebo Harmonic simulates contacts, dynamics, wheel motion, the GPU LiDAR,
  and the IMU.
- `ros_gz_bridge` transfers typed messages between Gazebo Transport and ROS 2.
- `robot_state_publisher` combines the URDF with `/joint_states` to publish the
  link transforms below `base_link`.
- RViz displays the robot, TF tree, wheel odometry, grid, and LiDAR scan.
- `teleop_twist_keyboard` publishes velocity commands on `/cmd_vel`.
- In navigation mode, `robot_localization` fuses wheel odometry and IMU data,
  AMCL localizes against the saved map, and Nav2 controls the robot.

## Runtime data flow

```text
teleop_twist_keyboard
  └── /cmd_vel (geometry_msgs/msg/Twist)
        └── ROS_TO_GZ bridge
              └── Gazebo DiffDrive system

Gazebo DiffDrive system
  ├── /odom (nav_msgs/msg/Odometry)
  └── /tf (tf2_msgs/msg/TFMessage: odom → base_link)

Gazebo JointStatePublisher
  └── /joint_states (sensor_msgs/msg/JointState)
        └── robot_state_publisher
              └── base_link → wheel, caster, and lidar transforms

Gazebo GPU LiDAR
  └── /scan (sensor_msgs/msg/LaserScan, frame lidar_link)

Gazebo IMU
  └── /imu/data (sensor_msgs/msg/Imu, frame imu_link)

Gazebo clock
  └── /clock (rosgraph_msgs/msg/Clock)
```

## Frames

```text
odom
└── base_link
    ├── left_wheel_link
    ├── right_wheel_link
    ├── caster_fork_link
    │   └── caster_wheel_link
    ├── lidar_link
    └── imu_link
```

RViz uses `odom` as its default fixed frame. Wheel odometry is expected to
drift during wheel slip or while the robot is commanded into an immovable
obstacle. For a robot-relative diagnostic view, temporarily use `base_link`
as the RViz fixed frame.

## Topic bridge directions

| Topic | ROS type | Direction | Nominal rate |
| --- | --- | --- | ---: |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo | On keypress |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS | 50 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | 50 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS | 10 Hz |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo → ROS | 100 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS | 50 Hz |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | Simulation rate |

## Autonomous navigation data flow

Navigation uses a dedicated bridge configuration that does not bridge Gazebo's
`/tf`, preventing competing transform publishers.

```text
/odom + /imu/data → EKF → /odometry/filtered + odom → base_link
/map + /scan + filtered odometry → AMCL → map → odom
Nav2 costmaps → planner → controller → velocity smoother
velocity smoother → /cmd_vel → Gazebo DiffDrive
```

Its frame tree is `map → odom → base_link`, followed by the robot links from
`robot_state_publisher`.

## Autonomous exploration data flow

Exploration replaces the saved-map/AMCL pair with SLAM Toolbox in mapping
mode. The live `/map` feeds Nav2's static global costmap and the in-repo
frontier coordinator:

```text
/scan + /odometry/filtered → SLAM Toolbox → /map + map → odom
/map → frontier extraction → ComputePathToPose → NavigateToPose → /cmd_vel
Gazebo /world/navigation_arena/pose/info → /ground_truth/poses
rendered SDF + truth poses + final /map → benchmark metrics
```

Coverage is a completion gate rather than an early-stop trigger. The explorer
continues while a reachable frontier exists. When frontiers appear exhausted,
it waits for three distinct SLAM map analyses and revalidates every temporarily
excluded frontier through the planner. Completion needs no mandatory sensor
spin. Below-target recovery is bounded, with at most one spin per location;
exhaustion saves as incomplete by default. Success requires at least 98%
coverage and no reachable frontier. Finalization cancels actions, confirms a
stationary robot and saves/scores one immutable map snapshot in a worker.
The existing direct PGM/YAML writer avoids a blocking SLAM save-service call.
Every run gets a durable unique folder, JSON metrics and Markdown report.
Transient-local final-map and summary-marker publishers keep results visible
in RViz while the robot holds zero velocity. The exploration smoother feeds
`/cmd_vel_exploration`; the mission gates it onto `/cmd_vel`, dropping motion
commands once finalization starts even if a cancellation response is delayed.
Batch mode can shut down instead;
shutdown requests are guarded against duplicate events. A saved-map run replaces
SLAM Toolbox with `map_server` and AMCL so
the same generated world can accept arbitrary RViz `2D Goal Pose` targets.

The truth stream is bridged as a `geometry_msgs/msg/PoseArray`. In the seeded
static arena, the robot model has a deterministic pose-array index equal to
`6 + obstacle_count` (ground plane, four walls, divider, then generated
obstacles). This avoids relying on the ROS bridge's name-dropping
`Pose_V → TFMessage` conversion while retaining authoritative simulator pose
data for distance and map scoring.
