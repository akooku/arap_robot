#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the 4-wheeled differential drive robot.

This script creates a launch description that starts the necessary controllers
for operating the differential drive robot in a specific sequence.

Launched Controllers:
    1. Joint State Broadcaster: Publishes joint states to /joint_states
    2. Differential Drive Controller: Controls the robot's differential drive movements via ~/cmd_vel

:author: Ako Eyo Oku
:date: March 9, 2025
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
 
 
def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.
 
    The function creates a launch sequence that ensures controllers are started
    in the correct order.
 
    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """
    # Start diff drive controller
    start_diff_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )
 
    # Start joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
 
    # Add delay to joint state broadcaster
    delayed_start = TimerAction(
        period=20.0,
        actions=[start_joint_state_broadcaster_cmd]
    )
 
    # Register event handler for sequencing
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_diff_drive_controller_cmd]))
 
    # Create and populate the launch description
    ld = LaunchDescription()
 
    # Add the actions to the launch description in sequence
    ld.add_action(delayed_start)
    ld.add_action(load_joint_state_broadcaster_cmd)