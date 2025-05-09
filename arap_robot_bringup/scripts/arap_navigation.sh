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

# Check if SLAM argument is provided
if [ "$1" = "slam" ]; then
    echo "🚀 Launching simulation for ARAP Robot with SLAM..."
    SLAM_ARG="slam:=True"
else
    echo "🚀 Launching simulation for ARAP Robot without SLAM..."
    SLAM_ARG="slam:=False"
fi

# For cafe.world -> z:=0.20
# For house.world -> z:=0.05
# To change Gazebo camera pose: gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

ros2 launch arap_robot_bringup arap_navigation.launch.py \
   autostart:=true \
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
   "$SLAM_ARG" &

echo "⏳ Waiting 25 seconds for simulation to initialize..."
sleep 25

# Optional: Adjust camera in Ignition Gazebo
# echo "🎥 Setting camera view in Gazebo..."
# gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req \
# "pose: {position: {x: 0.0, y: -2.0, z: 2.0}, orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

# Keep the script alive until interrupted
wait