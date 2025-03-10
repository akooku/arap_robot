/**
 * @file differential_drive_controller.cpp
 * @brief Implementation of the ROS 2 controller for a 4-wheeled differential drive robot
 *
 * This controller handles the kinematics of a differential drive robot,
 * converting velocity commands into wheel velocities while handling different
 * wheel radii and providing odometry information.
 *
 * Inverse Kinematics Equations (all angular velocities in radians/second):
 * ω_left = (vx - ωz * (W / 2)) / R
 * ω_right = (vx + ωz * (W / 2)) / R
 *
 * Where:
 * W = distance between left and right wheels (meters)
 * R = wheel radius (meters)
 * vx = linear x velocity (m/s)
 * ωz = angular velocity (rad/s)
 *
 * Subscription Topics:
 *     ~/cmd_vel (geometry_msgs/msg/TwistStamped): Velocity commands for the robot
 *
 * Publishing Topics:
 *     ~/odom (nav_msgs/msg/Odometry): Odometry information from wheel encoders
 *     /tf (tf2_msgs/msg/TFMessage): Transform between odom and base_link frames
 *
 * @author Modified for differential drive by Ako Eyo Oku
 */

    #include <memory>
    #include <optional>
    #include <queue>
    #include <string>
    #include <vector>

    #include "differential_drive_controller/differential_drive_controller.hpp"
    #include "hardware_interface/types/hardware_interface_type_values.hpp"
    #include "lifecycle_msgs/msg/state.hpp"
    #include "rclcpp/logging.hpp"
    #include "tf2/LinearMath/Quaternion.h"

    namespace
    {
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
    constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
    }  // namespace

    namespace differential_drive_controller
    {
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;
    using lifecycle_msgs::msg::State;

    DifferentialDriveController::DifferentialDriveController()
    : controller_interface::ControllerInterface()
    {
    }

    const char * DifferentialDriveController::feedback_type() const
    {
    return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
    }

    controller_interface::CallbackReturn DifferentialDriveController::on_init()
    {
    try
    {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration DifferentialDriveController::command_interface_configuration() const
    {
    std::vector<std::string> conf_names;
    conf_names.reserve(4);  // 4 wheels need independent configurations

    conf_names.push_back(params_.front_left_joint_name + "/" + HW_IF_VELOCITY);
    conf_names.push_back(params_.front_right_joint_name + "/" + HW_IF_VELOCITY);
    conf_names.push_back(params_.back_left_joint_name + "/" + HW_IF_VELOCITY);
    conf_names.push_back(params_.back_right_joint_name + "/" + HW_IF_VELOCITY);

    return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    InterfaceConfiguration DifferentialDriveController::state_interface_configuration() const
    {
    std::vector<std::string> conf_names;
    conf_names.reserve(4);

    conf_names.push_back(params_.front_left_joint_name + "/" + feedback_type());
    conf_names.push_back(params_.front_right_joint_name + "/" + feedback_type());
    conf_names.push_back(params_.back_left_joint_name + "/" + feedback_type());
    conf_names.push_back(params_.back_right_joint_name + "/" + feedback_type());

    return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type DifferentialDriveController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
    {
    auto logger = get_node()->get_logger();
    if (get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
    {
        if (!is_halted)
        {
        halt();
        is_halted = true;
        }
        return controller_interface::return_type::OK;
    }

    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get([&last_command_msg](const auto& msg) {
        last_command_msg = msg;
      });      

    if (last_command_msg == nullptr)
    {
        RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - last_command_msg->header.stamp;
    if (age_of_last_command > cmd_vel_timeout_)
    {
        last_command_msg->twist.linear.x = 0.0;
        last_command_msg->twist.angular.z = 0.0;
    }

    Twist command = *last_command_msg;
    double & linear_command_x = command.twist.linear.x;
    double & angular_command = command.twist.angular.z;

    previous_update_timestamp_ = time;

    // Apply speed limits
    auto & last_command = previous_commands_.back().twist;
    limiter_linear_x_.limit(
        linear_command_x, last_command.linear.x, previous_commands_.front().twist.linear.x, period.seconds());
    limiter_angular_.limit(
        angular_command, last_command.angular.z, previous_commands_.front().twist.angular.z, period.seconds());

    previous_commands_.pop();
    previous_commands_.emplace(command);

    if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
    {
        auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
        limited_velocity_command.header.stamp = time;
        limited_velocity_command.twist = command.twist;
        realtime_limited_velocity_publisher_->unlockAndPublish();
    }

    // Compute wheel velocities using differential drive kinematics
    const double W = params_.wheel_separation;
    const double R = params_.wheel_radius;

    const double left_wheel_velocity = (2 * linear_command_x - angular_command * W) / (2 * R);
    const double right_wheel_velocity = (2 * linear_command_x + angular_command * W) / (2 * R);

    // Apply velocity to all left-side wheels
    if (!wheel_handles_[FRONT_LEFT].velocity->get().set_value(left_wheel_velocity))
    {
        RCLCPP_ERROR(logger, "Failed to set front left wheel velocity");
    }
    if (!wheel_handles_[BACK_LEFT].velocity->get().set_value(left_wheel_velocity))
    {
        RCLCPP_ERROR(logger, "Failed to set back left wheel velocity");
    }

    // Apply velocity to all right-side wheels
    if (!wheel_handles_[FRONT_RIGHT].velocity->get().set_value(right_wheel_velocity))
    {
        RCLCPP_ERROR(logger, "Failed to set front right wheel velocity");
    }
    if (!wheel_handles_[BACK_RIGHT].velocity->get().set_value(right_wheel_velocity))
    {
        RCLCPP_ERROR(logger, "Failed to set back right wheel velocity");
    }

    return controller_interface::return_type::OK;
    }

    void DifferentialDriveController::halt()
    {
    for (auto & wheel_handle : wheel_handles_)
    {
        if (wheel_handle.velocity)
        {
        if (!wheel_handle.velocity->get().set_value(0.0))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to halt wheel velocity");
        }
        }
    }
    }

    }  // namespace differential_drive_controller

    #include "class_loader/register_macro.hpp"

    CLASS_LOADER_REGISTER_CLASS(
    differential_drive_controller::DifferentialDriveController,
    controller_interface::ControllerInterface)