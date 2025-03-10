/**
 * @file odometry.hpp
 * @brief Odometry calculations for a 4-wheeled differential drive robot
 *
 * This class handles odometry calculations for a differential drive robot
 * with four wheels (two on each side). It computes position and velocity
 * based on the average movement of the left and right wheel pairs.
 *
 * @author Addison Sears-Collins (modified for 4-wheeled differential drive)
 * @date March 9, 2025
 */

 #ifndef DIFFERENTIAL_DRIVE_CONTROLLER__ODOMETRY_HPP_
 #define DIFFERENTIAL_DRIVE_CONTROLLER__ODOMETRY_HPP_
 
 #include <cmath>
 #include "rclcpp/time.hpp"
 
 #if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
 #include "rcpputils/rolling_mean_accumulator.hpp"
 #else
 #include "rcppmath/rolling_mean_accumulator.hpp"
 #endif
 
 namespace differential_drive_controller
 {
 class Odometry
 {
 public:
   /**
    * @brief Constructor for differential drive odometry
    * @param velocity_rolling_window_size Window size for velocity averaging
    */
   explicit Odometry(size_t velocity_rolling_window_size = 10);
 
   /**
    * @brief Initialize odometry at the specified time
    * @param time Current time
    */
   void init(const rclcpp::Time & time);
 
   /**
    * @brief Update odometry from wheel positions
    * @param front_left_pos Front left wheel position (radians)
    * @param back_left_pos Rear left wheel position (radians)
    * @param right_front_pos Front right wheel position (radians)
    * @param back_right_pos Rear right wheel position (radians)
    * @param time Current time
    * @return True if update was successful
    */
   bool update(
     double front_left_pos, double back_left_pos,
     double right_front_pos, double back_right_pos,
     const rclcpp::Time & time);
 
   /**
    * @brief Update odometry from wheel velocities
    * @param front_left_vel Front left wheel velocity (rad/s)
    * @param back_left_vel Rear left wheel velocity (rad/s)
    * @param front_right_vel Front right wheel velocity (rad/s)
    * @param back_right_vel Rear right wheel velocity (rad/s)
    * @param time Current time
    * @return True if update was successful
    */
   bool updateFromVelocity(
     double front_left_vel, double back_left_vel,
     double front_right_vel, double back_right_vel,
     const rclcpp::Time & time);
 
   /**
    * @brief Update odometry using velocity commands (open loop)
    * @param linear_x Linear velocity in x direction (m/s)
    * @param angular Angular velocity (rad/s)
    * @param time Current time
    */
   void updateOpenLoop(double linear_x, double angular, const rclcpp::Time & time);
 
   /**
    * @brief Reset odometry to initial state
    */
   void resetOdometry();
 
   // Getters for odometry values
   double getX() const { return x_; }
   double getY() const { return y_; }
   double getHeading() const { return heading_; }
   double getLinearX() const { return linear_x_; }
   double getAngular() const { return angular_; }
 
   /**
    * @brief Set wheel parameters for odometry calculations
    * @param wheel_separation Distance between left and right wheels
    * @param wheel_radius Radius of the wheels
    */
   void setWheelParams(double wheel_separation, double wheel_radius);
 
   /**
    * @brief Set size of velocity rolling window
    * @param velocity_rolling_window_size Window size for velocity averaging
    */
   void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);
 
 private:
 #if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
   using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
 #else
   using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
 #endif
 
   /**
    * @brief Integrate motion using Runge-Kutta 2nd order method
    * @param linear_x Linear velocity in x direction
    * @param angular Angular velocity
    */
   void integrateRungeKutta2(double linear_x, double angular);
 
   /**
    * @brief Integrate motion using exact method
    * @param linear_x Linear velocity in x direction
    * @param angular Angular velocity
    */
   void integrateExact(double linear_x, double angular);
 
   /**
    * @brief Reset velocity accumulators
    */
   void resetAccumulators();
 
   // Current timestamp:
   rclcpp::Time timestamp_;
 
   // Current pose:
   double x_;        // [m]
   double y_;        // [m]
   double heading_;  // [rad]
 
   // Current velocity:
   double linear_x_;  // [m/s]
   double angular_;   // [rad/s]
 
   // Wheel kinematic parameters:
   double wheel_separation_;  // Distance between left and right wheels
   double wheel_radius_;      // Radius of wheels
 
   // Previous wheel positions:
   double front_left_wheel_old_pos_;
   double front_right_wheel_old_pos_;
   double back_left_wheel_old_pos_;
   double back_right_wheel_old_pos_;
 
   // Rolling mean accumulators for the velocities:
   size_t velocity_rolling_window_size_;
   RollingMeanAccumulator linear_x_accumulator_;
   RollingMeanAccumulator angular_accumulator_;
 };
 
 }  // namespace differential_drive_controller
 #endif  // DIFFERENTIAL_DRIVE_CONTROLLER__ODOMETRY_HPP_ 