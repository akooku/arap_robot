/**
 * @file differential_drive_controller.hpp
 * @brief ROS 2 controller for differential drive mobile robots
 *
 * This controller handles the kinematics of a differential drive robot with four wheels.
 * It subscribes to velocity commands and converts them into individual wheel velocities,
 * supporting forward and rotational movement.
 *
 * @author Addison Sears-Collins (modified for differential drive by ChatGPT)
 * @date March 8, 2025
 */

 #ifndef DIFFERENTIAL_DRIVE_CONTROLLER__DIFFERENTIAL_DRIVE_CONTROLLER_HPP_
 #define DIFFERENTIAL_DRIVE_CONTROLLER__DIFFERENTIAL_DRIVE_CONTROLLER_HPP_
 
 #include <chrono>
 #include <cmath>
 #include <memory>
 #include <optional>
 #include <queue>
 #include <string>
 #include <vector>
 #include <array>
 
 // ROS 2 Controller interface
 #include "controller_interface/controller_interface.hpp"
 // Custom headers for differential drive functionality
 #include "differential_drive_controller/odometry.hpp"
 #include "differential_drive_controller/speed_limiter.hpp"
 #include "differential_drive_controller/visibility_control.h"
 // ROS 2 message types
 #include "geometry_msgs/msg/twist_stamped.hpp"
 #include "nav_msgs/msg/odometry.hpp"
 #include "odometry.hpp"
 // ROS 2 lifecycle and utilities
 #include "rclcpp_lifecycle/state.hpp"
 #include "realtime_tools/realtime_box.hpp"
 #include "realtime_tools/realtime_publisher.hpp" 
 #include "tf2_msgs/msg/tf_message.hpp"
 
 // Auto-generated parameters
 #include <differential_drive_controller/differential_drive_controller_parameters.hpp>
 
 namespace differential_drive_controller
 {
 
 // Wheel position indices
 enum WheelIndex : size_t
 {
   FRONT_LEFT = 0,
   BACK_LEFT = 1,
   FRONT_RIGHT = 2,
   BACK_RIGHT = 3
 };
 
 /**
  * @brief Controller class for differential drive robots
  *
  * This class implements a controller for robots with differential drive wheels,
  * allowing for linear and rotational movement based on velocity commands.
  */
 class DifferentialDriveController : public controller_interface::ControllerInterface
 {
   using Twist = geometry_msgs::msg::TwistStamped;
 
 public:
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   DifferentialDriveController();
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::InterfaceConfiguration command_interface_configuration() const override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::InterfaceConfiguration state_interface_configuration() const override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::return_type update(
     const rclcpp::Time & time, const rclcpp::Duration & period) override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::CallbackReturn on_init() override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::CallbackReturn on_configure(
     const rclcpp_lifecycle::State & previous_state) override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::CallbackReturn on_activate(
     const rclcpp_lifecycle::State & previous_state) override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::CallbackReturn on_deactivate(
     const rclcpp_lifecycle::State & previous_state) override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::CallbackReturn on_cleanup(
     const rclcpp_lifecycle::State & previous_state) override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::CallbackReturn on_error(
     const rclcpp_lifecycle::State & previous_state) override;
 
   DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
   controller_interface::CallbackReturn on_shutdown(
     const rclcpp_lifecycle::State & previous_state) override;
 
 protected:
   struct WheelHandle
   {
     std::optional<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> feedback{};
     std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> velocity{};
 
     WheelHandle() = default;
 
     WheelHandle(
       const hardware_interface::LoanedStateInterface & feedback_handle,
       hardware_interface::LoanedCommandInterface & velocity_handle)
       : feedback(std::cref(feedback_handle)),
         velocity(std::ref(velocity_handle))
    {
    }
   };
 
   const char * feedback_type() const;
   controller_interface::CallbackReturn configure_wheel(
     const std::string & wheel_name,
     size_t wheel_index);
 
   std::array<WheelHandle, 4> wheel_handles_;
   std::shared_ptr<ParamListener> param_listener_;
   Params params_;
   Odometry odometry_;
 
   std::chrono::milliseconds cmd_vel_timeout_{500};
   std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
   std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
     realtime_odometry_publisher_ = nullptr;
 
   std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
   std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
     realtime_odometry_transform_publisher_ = nullptr;
 
   bool subscriber_is_active_ = false;
   rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
 
   realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};
   std::queue<Twist> previous_commands_;
 
   SpeedLimiter limiter_linear_x_;
   SpeedLimiter limiter_angular_;
 
   bool publish_limited_velocity_ = false;
   std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
   std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
     nullptr;
 
   rclcpp::Time previous_update_timestamp_{0};
   double publish_rate_ = 50.0;
   rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
   rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
 
   bool is_halted = false;
 
   bool reset();
   void halt();
 };
 }  // namespace differential_drive_controller
 
 #endif  // DIFFERENTIAL_DRIVE_CONTROLLER__DIFFERENTIAL_DRIVE_CONTROLLER_HPP_ 