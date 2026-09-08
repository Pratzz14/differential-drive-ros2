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
