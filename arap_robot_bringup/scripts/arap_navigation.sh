#!/bin/bash
# Single script to launch the ARAP Robot in Gazebo with EKF and RViz

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

trap 'cleanup' SIGINT SIGTERM

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM
 
# Create a temporary map file for SLAM mode
if [ "$1" = "slam" ]; then
  echo "Running in SLAM mode - creating a temporary map file"
  mkdir -p /tmp/maps
  echo "image: /tmp/maps/dummy.pgm
resolution: 0.05
origin: [-0.50, -0.20, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196" > /tmp/maps/dummy.yaml
  
  # Create an empty 10x10 PGM file
  echo "P5
  100 100
  255" > /tmp/maps/dummy.pgm
  for i in {1..10000}; do
    printf "\xff" >> /tmp/maps/dummy.pgm
  done
  
  SLAM_ARG="slam:=true"
  MAP_ARG="map:=/tmp/maps/dummy.yaml"
else
  SLAM_ARG="slam:=false"
  MAP_ARG="map:=/home/ubuntu/ros2_ws/src/arap_robot/arap_robot_navigation/maps/cafe_map.yaml"
fi  

# Add this before launching your main system
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock &
sleep 5

# For cafe.world -> z:=0.20
# For house.world -> z:=0.05
# To change Gazebo camera pose: gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

echo "🚀 Launching simulation for ARAP Robot..."
ros2 launch arap_robot_bringup arap_navigation.launch.py \
   world:=cafe \
   use_sim_time:=true \
   use_rviz:=true \
   use_robot_state_pub:=true \
   z:=0.20 \
   x:=0.0 \
   y:=0.0 \
   roll:=0.0 \
   pitch:=0.0 \
   yaw:=0.0 \
   "$SLAM_ARG" \
  "$MAP_ARG" &

echo "⏳ Waiting 25 seconds for simulation to initialize..."
sleep 25

# Optional: Adjust camera in Ignition Gazebo
# echo "🎥 Setting camera view in Gazebo..."
# gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req \
# "pose: {position: {x: 0.0, y: -2.0, z: 2.0}, orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

# Keep the script alive until interrupted
wait