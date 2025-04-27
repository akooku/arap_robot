#!/usr/bin/env python3
"""
@file cmd_vel_relay.py
@brief Relay node that converts Twist messages to TwistStamped messages

This program subscribes to velocity commands published as Twist messages and
republishes them as TwistStamped messages. This is useful when interfacing
between different ROS2 nodes that expect different message types for velocity
commands.

Subscription Topics:
    /cmd_vel (geometry_msgs/msg/Twist): Raw velocity commands

Publishing Topics:
    /mecanum_drive_controller/cmd_vel (geometry_msgs/msg/TwistStamped):
    Timestamped velocity commands

@author Ako Oku
@date April 4, 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelRelay(Node):
    """
    A ROS2 node that relays velocity commands between different message types.

    This class subscribes to Twist messages and republishes them as
    TwistStamped messages with a timestamp and frame ID.

    NOTE: May create a feedback loop as it subscribes to the same topic it publishes to.
    """

    def __init__(self):
        super().__init__('cmd_vel_relay')

        # Subscription to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10  # queue size
        )

        # Publisher for /mecanum_drive_controller/cmd_vel topic
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10  # queue size
        )

        self.get_logger().info('Velocity relay node started')

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for incoming Twist messages. Converts to TwistStamped and publishes.
        """
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg

        self.publisher.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
