#!/usr/bin/env python3
"""
Simple Square Differential Drive Controller

This program creates a ROS 2 node that publishes velocity commands to make a
differential drive robot move in a square pattern using a simple time-based approach.

Publishing Topics:
    /cmd_vel (geometry_msgs/Twist): Velocity commands for the robot's motion
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareController(Node):
    def __init__(self):
        super().__init__('square_controller')
        self.get_logger().info("Square controller node started!")
        
        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize variables
        self.current_side = 0     # Which side of square we're on (0-3)
        self.elapsed_time = 0.0   # Time spent on current side
        self.robot_speed = 0.2    # Speed in meters/second
        self.turn_speed = 5.0     # Angular speed in radians/second - VERY FAST TURNING
        self.side_length = 2.0    # Length of each side in meters
        
        # Calculate timing
        self.time_per_side = self.side_length / self.robot_speed  # Time to complete one side
        # Increased turning time to ensure complete 90-degree turns
        self.time_per_turn = 5  # Time to complete a 90-degree turn (~100-115 degrees to be safe)
        
        # State can be "forward" or "turning"
        self.state = "forward"
        
        # Create timer that calls our control function every 100ms
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def stop_robot(self):
        """Function to stop the robot"""
        msg = Twist()
        self.publisher.publish(msg)  # All velocities default to 0
        
        # Publish multiple times to ensure it's received
        for _ in range(3):
            self.publisher.publish(msg)
            time.sleep(0.01)
    
    def timer_callback(self):
        """Timer callback for controlling the robot"""
        # Create velocity command message
        msg = Twist()
        
        if self.state == "forward":
            # Set linear velocity based on which side of the square we're on
            if self.current_side == 0:    # Moving forward (positive X)
                msg.linear.x = self.robot_speed
            elif self.current_side == 1:  # Moving left (would be Y in mecanum, but we turn instead)
                msg.linear.x = self.robot_speed
            elif self.current_side == 2:  # Moving backward (negative X)
                msg.linear.x = self.robot_speed
            elif self.current_side == 3:  # Moving right (would be Y in mecanum, but we turn instead)
                msg.linear.x = self.robot_speed
            
            # Publish the velocity command
            self.publisher.publish(msg)
            
            # Update time tracking
            self.elapsed_time += 0.1  # 100ms in seconds
            
            # Check if we've completed the current side
            if self.elapsed_time >= self.time_per_side:
                self.get_logger().info(f"Completed side {self.current_side + 1}")
                self.stop_robot()
                # Small delay after stopping to ensure the robot fully stops
                time.sleep(0.3)
                self.state = "turning"
                self.elapsed_time = 0.0
                self.get_logger().info(f"Starting 90-degree turn...")
        
        elif self.state == "turning":
            # Always turn counterclockwise (positive angular Z) with very high speed
            msg.angular.z = self.turn_speed
            
            # Publish the velocity command
            self.publisher.publish(msg)
            
            # Log turning progress more frequently
            if int(self.elapsed_time / 0.1) % 5 == 0:  # Log every ~0.5 seconds
                self.get_logger().info(f"Turning... {self.elapsed_time:.1f}s / {self.time_per_turn:.1f}s")
            
            # Update time tracking
            self.elapsed_time += 0.1  # 100ms in seconds
            
            # Check if we've completed the turn
            if self.elapsed_time >= self.time_per_turn:
                self.get_logger().info(f"Completed turn, moving to side {(self.current_side + 2) % 4 + 1}")
                self.stop_robot()
                # Small delay after stopping to ensure the robot fully stops
                time.sleep(0.3)
                self.state = "forward"
                self.current_side = (self.current_side + 1) % 4  # Move to next side
                self.elapsed_time = 0.0
                
                # Check if we've completed the square
                if self.current_side == 0:
                    self.get_logger().info("Square pattern completed!")

def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*50)
    print("SIMPLE SQUARE CONTROLLER")
    print("This node will drive the robot in a square pattern")
    print("="*50 + "\n")
    
    controller = SquareController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Node stopped by user")
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()