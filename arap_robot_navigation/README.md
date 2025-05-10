# ARAP Robot Navigation

This package provides the navigation stack configuration for the ARAP robot using Nav2. It includes all necessary parameters and launch files for autonomous navigation.

## Package Structure
```
arap_robot_navigation/
├── config/
│   ├── arap_nav2_default_params.yaml  # All navigation parameters
│   └── mapper_params_online_async.yaml # SLAM parameters
├── launch/
│   └── navigation.launch.py           # Navigation launch file
├── maps/                             # Default maps
└── rviz/                             # RViz configuration
```

## Configuration Parameters

### AMCL Parameters
```yaml
amcl:
  ros__parameters:
    # Motion Model Parameters
    alpha1: 0.2  # Rotation noise from rotation
    alpha2: 0.2  # Rotation noise from translation
    alpha3: 0.2  # Translation noise from translation
    alpha4: 0.2  # Translation noise from rotation
    alpha5: 0.2  # Translation noise from translation (rotation)

    # Particle Filter Parameters
    min_particles: 500    # Minimum number of particles
    max_particles: 2000   # Maximum number of particles
    pf_err: 0.05         # Error in particle filter
    pf_z: 0.99           # Measurement model z-score

    # Laser Model Parameters
    laser_model_type: "likelihood_field"  # Type of laser model
    laser_max_range: 100.0               # Maximum range of laser
    laser_min_range: -1.0                # Minimum range of laser
    laser_likelihood_max_dist: 2.0       # Maximum distance for likelihood
    beam_skip_distance: 0.5              # Distance to skip beams
    beam_skip_threshold: 0.3             # Threshold for beam skipping
    beam_skip_error_threshold: 0.9       # Error threshold for beam skipping

    # Recovery Parameters
    recovery_alpha_slow: 0.001  # Slow recovery rate
    recovery_alpha_fast: 0.1    # Fast recovery rate
    resample_interval: 1        # Resampling interval
```

### DWB Controller Parameters
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # Velocity Limits
      min_vel_x: 0.0      # Minimum linear velocity
      max_vel_x: 0.26     # Maximum linear velocity
      max_vel_theta: 1.0  # Maximum angular velocity
      
      # Acceleration Limits
      acc_lim_x: 2.5      # Linear acceleration limit
      acc_lim_theta: 3.2  # Angular acceleration limit
      
      # Trajectory Generation
      vx_samples: 20      # Number of linear velocity samples
      vtheta_samples: 40  # Number of angular velocity samples
      sim_time: 1.7       # Time to simulate forward
      
      # Cost Function Weights
      PathAlign.scale: 32.0        # Weight for path alignment
      GoalAlign.scale: 24.0        # Weight for goal alignment
      PathDist.scale: 32.0         # Weight for path distance
      GoalDist.scale: 24.0         # Weight for goal distance
      RotateToGoal.scale: 32.0     # Weight for rotation to goal
```

### Costmap Parameters
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Update and Publishing
      update_frequency: 5.0    # How often to update the costmap
      publish_frequency: 2.0   # How often to publish the costmap
      
      # Costmap Size
      width: 3                # Width of the costmap in meters
      height: 3               # Height of the costmap in meters
      resolution: 0.05        # Resolution of the costmap in meters/cell
      
      # Robot Properties
      robot_radius: 0.22      # Radius of the robot for obstacle inflation
      
      # Inflation Layer
      inflation_layer:
        cost_scaling_factor: 3.0  # How quickly the cost decreases with distance
        inflation_radius: 0.55    # How far to inflate obstacles

global_costmap:
  global_costmap:
    ros__parameters:
      # Update and Publishing
      update_frequency: 1.0    # How often to update the costmap
      publish_frequency: 0.5   # How often to publish the costmap
      
      # Costmap Properties
      resolution: 0.05         # Resolution of the costmap in meters/cell
      track_unknown_space: true  # Whether to track unknown space
      
      # Obstacle Layer
      obstacle_layer:
        obstacle_range: 2.5    # Maximum range to mark obstacles
        raytrace_range: 3.0    # Maximum range to clear obstacles
        max_obstacle_height: 2.0  # Maximum height of obstacles
```

### Planner Parameters
```yaml
planner_server:
  ros__parameters:
    GridBased:
      tolerance: 0.5          # Goal tolerance in meters
      use_astar: false       # Whether to use A* instead of Dijkstra
      allow_unknown: true    # Whether to allow planning through unknown space
```

### Recovery Parameters
```yaml
recoveries_server:
  ros__parameters:
    cycle_frequency: 10.0    # How often to check for recovery
    max_rotational_vel: 1.0  # Maximum rotational velocity for recovery
    min_rotational_vel: 0.4  # Minimum rotational velocity for recovery
    rotational_acc_lim: 3.2  # Rotational acceleration limit for recovery
```

## Parameter Tuning Guide

### 1. AMCL Tuning
- Increase `max_particles` for more accurate localization but higher CPU usage
- Adjust `alpha1-5` based on your robot's odometry accuracy
- Tune `laser_likelihood_max_dist` based on your environment's complexity

### 2. Controller Tuning
- Adjust velocity limits based on your robot's capabilities
- Modify acceleration limits for smoother motion
- Tune cost function weights to prioritize different aspects of navigation

### 3. Costmap Tuning
- Adjust `update_frequency` based on your sensor update rate
- Modify `inflation_radius` based on your robot's size and safety requirements
- Tune `obstacle_range` and `raytrace_range` based on your sensor capabilities

### 4. Planner Tuning
- Adjust `tolerance` based on your navigation precision requirements
- Enable `use_astar` for faster planning in complex environments
- Set `allow_unknown` based on your environment's characteristics

## Configuration

### 1. Navigation Parameters (arap_nav2_default_params.yaml)
This single YAML file contains all navigation-related parameters:

```yaml
amcl:
  ros__parameters:
    # AMCL parameters
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_slow: 0.001
    recovery_alpha_fast: 0.1
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: /scan
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: false
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /pointcloud
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "sensor_msgs/PointCloud2"
          raytrace_range: 3.0
          raytrace_range: 3.0
          obstacle_range: 2.5
          inf_is_valid: false
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: false
  local_costmap_server:
    ros__parameters:
      use_sim_time: false

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        enabled: True
        obstacle_range: 2.5
        raytrace_range: 3.0
        max_obstacle_height: 2.0
        combination_method: 1
        footprint_clearing_enabled: true
        laser_scan_sources: scan
        pointcloud_sources: pointcloud
        observation_sources: scan pointcloud
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "sensor_msgs/LaserScan"
          raytrace_range: 3.0
          obstacle_range: 2.5
          inf_is_valid: false
          marking: true
          clearing: true
        pointcloud:
          topic: /pointcloud
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "sensor_msgs/PointCloud2"
          raytrace_range: 3.0
          obstacle_range: 2.5
          inf_is_valid: false
      static_layer:
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: false
  global_costmap_server:
    ros__parameters:
      use_sim_time: false
```

## Using Navigation

### 1. Launching Navigation
```bash
ros2 launch arap_robot_navigation navigation.launch.py
```

### 2. Setting Initial Pose
```bash
# Using RViz
# 1. Click "2D Pose Estimate" button
# 2. Click and drag on the map to set position and orientation
```

### 3. Sending Navigation Goals
```bash
# Using RViz
# 1. Click "2D Goal Pose" button
# 2. Click and drag on the map to set goal position and orientation
```

## Best Practices

1. **Parameter Tuning**
   - Adjust velocity limits
   - Tune costmap parameters
   - Configure recovery behaviors

2. **Performance Monitoring**
   - Monitor CPU usage
   - Check memory consumption
   - Verify update rates

3. **Safety Considerations**
   - Set appropriate safety margins
   - Configure obstacle detection
   - Test recovery behaviors

## Common Issues

1. **Navigation Problems**
   - Check costmap configuration
   - Verify sensor data
   - Monitor path planning

2. **Performance Issues**
   - Adjust update frequencies
   - Check sensor timeouts
   - Monitor CPU usage

3. **Localization Issues**
   - Verify initial pose
   - Check AMCL parameters
   - Monitor particle filter

## Contributing
Feel free to submit issues and enhancement requests!
