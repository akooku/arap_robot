# ARAP Robot Custom Feature

This package implements a custom node called `multimapmanager` that serves as a proof of concept for extending the ARAP robot system. It demonstrates how users can build their own packages on top of the system while maintaining compatibility with the core functionality.

## Package Structure
```
arap_robot_custom_feature/
├── arap_robot_custom_feature/
│   └── __init__.py            # Python package initialization
├── scripts/
│   └── multi_map_manager_node.py  # Multi-map manager node implementation
├── maps/                      # Directory for storing maps
├── test/                      # Test directory
└── setup.py                   # Python package setup file
```

## Configuration

### 1. Multi-map Parameters (multimap_params.yaml)
Configure the multi-map manager:

```yaml
multimap_manager:
  ros__parameters:
    enabled: true
    update_rate: 10.0
    map_switching:
      enabled: true
      transition_time: 1.0
    map_storage:
      directory: "maps"
      format: "yaml"
```

## Using the Multi-map Manager

### 1. Launching the Manager
```bash
ros2 run arap_robot_custom_feature multi_map_manager_node.py
```

### 2. Using the Manager in Your Code
```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Subscribe to map updates
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        # Subscribe to map switching status
        self.status_sub = self.create_subscription(
            String,
            'map_status',
            self.status_callback,
            10
        )

    def map_callback(self, msg):
        # Process map updates
        self.get_logger().info('Received map update')

    def status_callback(self, msg):
        # Process status updates
        self.get_logger().info(f'Map status: {msg.data}')
```

## Extending the System

### 1. Adding New Functionality
Create a new Python node in the `scripts` directory:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NewFeatureNode(Node):
    def __init__(self):
        super().__init__('new_feature_node')
        self.publisher = self.create_publisher(
            String,
            'new_feature_topic',
            10
        )
        self.subscription = self.create_subscription(
            String,
            'map_status',
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        # Process status updates
        self.get_logger().info(f'Received status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = NewFeatureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing

### 1. Unit Tests
```python
import unittest
import rclpy
from std_msgs.msg import String

class TestMultiMapManager(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_multimap')
        self.status_sub = self.node.create_subscription(
            String,
            'map_status',
            self.status_callback,
            10
        )
        self.last_status = None

    def status_callback(self, msg):
        self.last_status = msg

    def test_map_switching(self):
        # Test implementation
        pass

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
```

## Best Practices

1. **Code Organization**
   - Keep nodes modular
   - Use clear naming conventions
   - Document interfaces

2. **Parameter Management**
   - Use ROS 2 parameters
   - Provide default values
   - Document parameter purposes

3. **Testing**
   - Write unit tests
   - Test edge cases
   - Verify error handling

## Common Issues

1. **Integration Problems**
   - Check message types
   - Verify topic names
   - Check parameter names

2. **Performance Issues**
   - Monitor CPU usage
   - Check memory consumption
   - Optimize algorithms

3. **Dependency Issues**
   - Verify package dependencies
   - Check version compatibility
   - Update package.xml

## Contributing
Feel free to submit issues and enhancement requests!
