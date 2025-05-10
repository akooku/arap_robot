# ARAP Robot System Tests

This package contains system-level tests for the ARAP robot. It provides a framework for testing the robot's functionality in both simulation and real-world environments.

## Package Structure
```
arap_robot_system_tests/
├── config/
│   └── test_params.yaml        # Test configuration parameters
├── launch/
│   └── test.launch.py          # Test launch file
├── test/
│   ├── test_navigation.py      # Navigation tests
│   ├── test_localization.py    # Localization tests
│   └── test_sensors.py         # Sensor tests
└── worlds/
    └── test_world.world        # Test world for simulation
```

## Test Configuration

### 1. Test Parameters (test_params.yaml)
Configure test parameters:

```yaml
test:
  ros__parameters:
    navigation:
      timeout: 30.0
      goal_tolerance: 0.1
      test_points:
        - [1.0, 0.0, 0.0]
        - [0.0, 1.0, 0.0]
        - [-1.0, 0.0, 0.0]
    localization:
      timeout: 10.0
      position_tolerance: 0.05
      orientation_tolerance: 0.1
    sensors:
      timeout: 5.0
      update_rate: 10.0
```

## Writing Tests

### 1. Navigation Tests
Example test case:

```python
import unittest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class TestNavigation(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_navigation')
        self.nav_client = self.node.create_action_client(
            NavigateToPose,
            'navigate_to_pose'
        )

    def test_navigation_to_goal(self):
        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0

        # Send goal
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        # Check result
        self.assertTrue(future.result().accepted)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
```

### 2. Localization Tests
Example test case:

```python
import unittest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

class TestLocalization(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_localization')
        self.pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )
        self.last_pose = None

    def pose_callback(self, msg):
        self.last_pose = msg

    def test_localization_accuracy(self):
        # Wait for pose
        rclpy.spin_once(self.node, timeout_sec=5.0)
        self.assertIsNotNone(self.last_pose)
        
        # Check covariance
        covariance = self.last_pose.pose.covariance
        self.assertLess(covariance[0], 0.1)  # x position variance
        self.assertLess(covariance[7], 0.1)  # y position variance

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
```

### 3. Sensor Tests
Example test case:

```python
import unittest
import rclpy
from sensor_msgs.msg import LaserScan

class TestSensors(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_sensors')
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.last_scan = None

    def scan_callback(self, msg):
        self.last_scan = msg

    def test_laser_scan(self):
        # Wait for scan
        rclpy.spin_once(self.node, timeout_sec=5.0)
        self.assertIsNotNone(self.last_scan)
        
        # Check scan properties
        self.assertGreater(len(self.last_scan.ranges), 0)
        self.assertLess(self.last_scan.range_min, self.last_scan.range_max)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
```

## Running Tests

1. Run all tests:
```bash
colcon test --packages-select arap_robot_system_tests
```

2. Run specific test:
```bash
python3 -m pytest test/test_navigation.py -v
```

3. Run with coverage:
```bash
colcon test --packages-select arap_robot_system_tests --coverage
```

## Best Practices

1. **Test Organization**
   - Group related tests
   - Use descriptive test names
   - Follow test naming conventions

2. **Test Coverage**
   - Test edge cases
   - Verify error handling
   - Check timeout scenarios

3. **Test Environment**
   - Use simulation when possible
   - Document test prerequisites
   - Clean up test resources

## Common Issues

1. **Test Failures**
   - Check test prerequisites
   - Verify test parameters
   - Review test logs

2. **Timing Issues**
   - Adjust timeouts
   - Check message rates
   - Verify node startup

3. **Resource Problems**
   - Clean up resources
   - Check memory usage
   - Monitor CPU load

## Contributing
Feel free to submit issues and enhancement requests!