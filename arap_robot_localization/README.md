# ARAP Robot Localization

This package provides the Extended Kalman Filter (EKF) localization implementation for the ARAP robot. It uses the robot's sensor data to estimate its position and orientation in the environment.

## Package Structure
```
arap_robot_localization/
├── config/
│   └── ekf.yaml              # EKF configuration parameters
├── launch/
│   └── ekf.launch.py         # EKF launch file
└── src/                      # Source files
```

## Configuration

### 1. EKF Parameters (ekf.yaml)
Configure the Extended Kalman Filter:

```yaml
ekf_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    transform_time_offset: 0.0
    transform_timeout: 0.0
    print_diagnostics: true
    debug: false
    publish_tf: true
    publish_acceleration: false
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /odom
    odom0_config: [true,  true,  false,
                  false, false, true,
                  true,  false, false,
                  false, false, true,
                  false, false, false]
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10
    odom0_nodelay: false
    odom0_remove_topics: true
```

## Using the Localization

### 1. Launching the EKF
```bash
ros2 launch arap_robot_localization ekf.launch.py
```

### 2. Monitoring Localization
```bash
# View the current pose estimate
ros2 topic echo /odometry/filtered

# View the transform tree
ros2 run tf2_tools view_frames
```

## Best Practices

1. **Sensor Configuration**
   - Ensure proper sensor calibration
   - Verify sensor data rates
   - Check sensor transformations

2. **Parameter Tuning**
   - Adjust process noise
   - Tune measurement noise
   - Configure sensor fusion

3. **Performance Monitoring**
   - Monitor CPU usage
   - Check memory consumption
   - Verify update rates

## Common Issues

1. **Localization Problems**
   - Check sensor data
   - Verify transformations
   - Monitor covariance

2. **Performance Issues**
   - Adjust update frequency
   - Check sensor timeouts
   - Monitor CPU usage

3. **Transform Issues**
   - Verify frame names
   - Check transform tree
   - Monitor transform delays

## Contributing
Feel free to submit issues and enhancement requests!
