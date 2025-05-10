#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from nav2_msgs.srv import LoadMap

class MultiMapManager(Node):
    def __init__(self):
        super().__init__('multi_map_manager')
        # List of maps and link points (could be loaded from a config file)
        self.maps = [
            {'name': 'building_a', 'yaml': '/home/aix/ros2_ws/src/arap_robot/arap_robot_custom_feature/maps/building_a.yaml', 'link_point': [1.0, 2.0, 0.0]},
            {'name': 'building_b', 'yaml': '/home/aix/ros2_ws/src/arap_robot/arap_robot_custom_feature/maps/building_b.yaml', 'link_point': [5.0, 8.0, 0.0]},
            # Add more maps as needed
        ]
        self.current_map_index = 0

        # Publisher for current map
        self.current_map_pub = self.create_publisher(String, '/current_map', 10)
        # Publisher for link point
        self.link_point_pub = self.create_publisher(Pose, '/map_link_point', 10)

        # Service to switch map
        self.srv = self.create_service(Trigger, '/switch_map', self.switch_map_callback)

        # Client for map_server's LoadMap service
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')

        self.publish_current_map()
        self.get_logger().info('MultiMapManager node started.')

    def publish_current_map(self):
        msg = String()
        msg.data = self.maps[self.current_map_index]['name']
        self.current_map_pub.publish(msg)
        self.get_logger().info(f"Current map: {msg.data}")

    def publish_link_point(self):
        pose = Pose()
        x, y, theta = self.maps[self.current_map_index]['link_point']
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        # Orientation as quaternion (theta in yaw)
        import math
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, theta)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self.link_point_pub.publish(pose)
        self.get_logger().info(f"Published link point: {x}, {y}, {theta}")

    def switch_map_callback(self, request, response):
        # Switch to next map in the list
        self.current_map_index = (self.current_map_index + 1) % len(self.maps)
        map_yaml = self.maps[self.current_map_index]['yaml']
        self.get_logger().info(f"Switching to map: {map_yaml}")

        # Wait for map_server service
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = "Map server service not available"
            return response

        # Call LoadMap service
        req = LoadMap.Request()
        req.map_url = map_yaml
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().result.success:
            self.get_logger().info("Map loaded successfully.")
            self.publish_current_map()
            self.publish_link_point()
            response.success = True
            response.message = f"Switched to map: {map_yaml}"
        else:
            self.get_logger().error("Failed to load map.")
            response.success = False
            response.message = "Failed to load map."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MultiMapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
