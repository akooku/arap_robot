#!/bin/bash
# Script to test basic odometry without navigation

echo "Starting robot with minimal components..."
ros2 launch arap_robot_gazebo sim.launch.py world:=empty &
sleep 10

echo "Publishing velocity commands to test odometry..."
# Send some velocity commands
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" &

echo "Monitoring odometry and transforms..."
# In another terminal, check the odometry output
ros2 topic echo /odom --no-arr &

# Check transform from odom to base_link
ros2 run tf2_ros tf2_echo odom base_link &

wait