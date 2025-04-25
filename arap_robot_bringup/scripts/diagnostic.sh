#!/bin/bash
# Save as ~/ros2_ws/src/arap_robot/diagnostic.sh and make it executable

echo "===== Checking TF Tree ====="
ros2 run tf2_tools view_frames

echo "===== Checking Node List ====="
ros2 node list

echo "===== Checking Controller State ====="
ros2 controller list

echo "===== Checking Topic List ====="
ros2 topic list | grep -E 'tf|joint|odom|cmd_vel|scan|costmap'

echo "===== Checking Transform Broadcasting ====="
ros2 topic echo /tf --field transforms[0].header.frame_id --no-arr --once

echo "===== Checking Map Publication ====="
ros2 topic echo /map/metadata --once