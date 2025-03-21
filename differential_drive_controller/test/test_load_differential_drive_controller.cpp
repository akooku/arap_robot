/**
 * @file test_load_differential_drive_controller.cpp
 * @brief Basic test to verify that the differential drive controller loads correctly
 *
 * This program tests if the differential drive controller can be loaded into the
 * ROS 2 Control framework. It uses the minimal robot URDF for testing and
 * verifies that loading the controller does not throw any exceptions.
 *
 * Test Components:
 *     - Initializes ROS 2
 *     - Creates a controller manager
 *     - Attempts to load the differential drive controller
 *     - Verifies no exceptions are thrown during loading
 *
 * @author Addison Sears-Collins
 * @date November 19, 2024
 */

// Google Mock for testing
#include <gmock/gmock.h>
// Standard library includes
#include <memory>
// ROS 2 Controller includes
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
// ROS 2 utilities
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
// Test assets
#include "ros2_control_test_assets/descriptions.hpp"

/**
 * @brief Test case to verify the differential drive controller can be loaded
 *
 * This test performs the following steps:
 * 1. Initializes ROS 2
 * 2. Creates a single-threaded executor
 * 3. Sets up a controller manager with minimal robot URDF
 * 4. Attempts to load the differential drive controller
 * 5. Verifies no exceptions occur during loading
 */
TEST(TestLoadDifferentialDriveController, load_controller)
{
  // Initialize ROS 2 without any command line arguments
  rclcpp::init(0, nullptr);

  // Create a single-threaded executor for running the controller
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Create the controller manager with a minimal robot description
  auto node = rclcpp::Node::make_shared("test_controller_manager");
  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::minimal_robot_urdf,
      node->get_node_clock_interface(),
      node->get_node_logging_interface()),
    executor,
    "test_controller_manager");

  // Attempt to load the controller and verify no exceptions are thrown
  ASSERT_NO_THROW(
    cm.load_controller(
      "test_differential_drive_controller",
      "differential_drive_controller/DifferentialDriveController"));

  // Clean up ROS 2
  rclcpp::shutdown();
}