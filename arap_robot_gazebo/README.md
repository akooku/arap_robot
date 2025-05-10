# ARAP Robot Gazebo

This package contains the Gazebo simulation configurations and launch files for the ARAP robot. It provides a simulated environment for testing and developing the robot's capabilities using the ROS 2 and Gazebo bridge.

## Package Structure
```
arap_robot_gazebo/
├── config/
│   ├── gazebo_bridge_params.yaml  # Gazebo bridge configuration parameters
│   └── ros_gz_bridge.yaml         # ROS 2 to Gazebo bridge parameters
├── launch/
│   └── sim.launch.py              # Main simulation launch file
└── worlds/
    └── default.world              # Default simulation world
```

## Configuration Files

### 1. Gazebo Bridge Parameters (gazebo_bridge_params.yaml)
Configure the ROS 2 to Gazebo bridge parameters:

```yaml
gazebo_bridge:
  ros__parameters:
    bridge:
      - topic_name: "/cmd_vel"
        ros_type_name: "geometry_msgs/msg/Twist"
        gz_type_name: "gz.msgs.Twist"
        direction: "ROS_TO_GZ"
      - topic_name: "/odom"
        ros_type_name: "nav_msgs/msg/Odometry"
        gz_type_name: "gz.msgs.Odometry"
        direction: "GZ_TO_ROS"
```

### 2. ROS-Gazebo Bridge Parameters (ros_gz_bridge.yaml)
Configure the bridge settings:

```yaml
ros_gz_bridge:
  ros__parameters:
    bridge:
      - topic_name: "/scan"
        ros_type_name: "sensor_msgs/msg/LaserScan"
        gz_type_name: "gz.msgs.LaserScan"
        direction: "GZ_TO_ROS"
      - topic_name: "/imu"
        ros_type_name: "sensor_msgs/msg/Imu"
        gz_type_name: "gz.msgs.IMU"
        direction: "GZ_TO_ROS"
```

## Using the Simulation

### 1. Launching the Simulation
```bash
ros2 launch arap_robot_gazebo sim.launch.py
```

This will:
- Start the Gazebo simulator
- Launch the ROS 2 to Gazebo bridge
- Spawn the robot in the default world
- Configure all necessary topics and parameters

### 2. Testing Robot Control
```bash
# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# View sensor data
ros2 topic echo /scan
ros2 topic echo /odom
```

## Best Practices

1. **Bridge Configuration**
   - Keep topic names consistent
   - Verify message type compatibility
   - Monitor bridge performance

2. **Simulation Performance**
   - Monitor CPU usage
   - Check bridge message rates
   - Optimize world complexity

## Common Issues

1. **Bridge Problems**
   - Check topic names
   - Verify message types
   - Monitor bridge status

2. **Performance Issues**
   - Reduce world complexity
   - Check bridge message rates
   - Monitor system resources

## Contributing
Feel free to submit issues and enhancement requests!