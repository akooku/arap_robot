#!/bin/bash

# 1. Launch the system (edit this to your actual launch command)
echo "Ensure you've launched the system!"

# 2. Wait for system to initialize
echo "Waiting for system to initialize..."
sleep 20  # Adjust as needed

# 3. List nodes and topics
echo "Listing active ROS 2 nodes:"
ros2 node list

echo "Listing important topics:"
ros2 topic list | grep -E '/scan|/odom|/cmd_vel|/map|/amcl_pose'

# 4. Capture TF tree
echo "Capturing TF tree..."
ros2 run tf2_tools view_frames
cp frames.pdf system_test_frames.pdf

# 5. (Optional) Save rqt_graph
echo "Please open rqt_graph and save the image manually for documentation."

# 6. (Optional) Record a rosbag for a short period
# ros2 bag record -o system_test_bag /scan /odom /cmd_vel /map /amcl_pose &

# 7. (Optional) Take a screenshot of RViz (if running in GUI)
# Use your OS screenshot tool or gnome-screenshot

# 8. Kill the launch after test
echo "System test complete. Check system_test_frames.pdf and logs."