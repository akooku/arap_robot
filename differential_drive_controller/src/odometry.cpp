/**
 * @file odometry.cpp
 * @brief Implementation of odometry calculations for 4-wheeled differential drive robots
 *
 * This implementation tracks the robot's position and velocity in 2D space,
 * based on **4-wheeled differential drive kinematics** (non-holonomic motion).
 *
 * Forward Kinematics Equations:
 * linear_x (m/s) = (R * (w_left + w_right)) / 2
 * angular (rad/s) = (R * (w_right - w_left)) / W
 *
 * Where:
 * R = wheel radius (meters)
 * W = distance between left and right wheels (meters)
 * w_left, w_right = average angular velocities of left and right wheels (radians/second)
 *
 * @author Modified for 4-wheel differential drive by Ako Eyo Oku
 * @date 09 March 2025
 */

 #include "differential_drive_controller/odometry.hpp"

 namespace differential_drive_controller
 {
 
 /**
  * @brief Constructor for Odometry class
  * @param velocity_rolling_window_size Size of window for velocity averaging
  */
 Odometry::Odometry(size_t velocity_rolling_window_size)
 : timestamp_(0.0),
   x_(0.0),           // Robot x position in meters
   y_(0.0),           // Robot y position in meters
   heading_(0.0),     // Robot heading in radians
   linear_x_(0.0),    // Forward/backward velocity in m/s
   angular_(0.0),     // Angular velocity in rad/s
   wheel_separation_(0.0),  // Distance between left and right wheels (meters)
   wheel_radius_(0.0),      // Radius of wheels (meters)
   // Previous wheel positions in radians
   front_left_wheel_old_pos_(0.0),
   front_right_wheel_old_pos_(0.0),
   back_left_wheel_old_pos_(0.0),
   back_right_wheel_old_pos_(0.0),
   velocity_rolling_window_size_(velocity_rolling_window_size),
   linear_x_accumulator_(velocity_rolling_window_size),
   angular_accumulator_(velocity_rolling_window_size)
 {
 }
 
 /**
  * @brief Initialize odometry with starting timestamp
  * @param time Current ROS time
  */
 void Odometry::init(const rclcpp::Time & time)
 {
   resetAccumulators();
   timestamp_ = time;
 }
 
 /**
  * @brief Update odometry using wheel position measurements
  * @param front_left_pos Front left wheel position (radians)
  * @param front_right_pos Front right wheel position (radians)
  * @param back_left_pos Back left wheel position (radians)
  * @param back_right_pos Back right wheel position (radians)
  * @param time Current ROS time
  * @return true if update successful, false if time interval too small
  */
 bool Odometry::update(
   double front_left_pos, double front_right_pos,
   double back_left_pos, double back_right_pos,
   const rclcpp::Time & time)
 {
   // Compute time step
   const double dt = time.seconds() - timestamp_.seconds();
 
   if (dt < 0.0001)
   {
     return false;
   }
 
   // Compute wheel velocities (rad/s)
   const double front_left_wheel_vel = (front_left_pos - front_left_wheel_old_pos_) / dt;
   const double front_right_wheel_vel = (front_right_pos - front_right_wheel_old_pos_) / dt;
   const double back_left_wheel_vel = (back_left_pos - back_left_wheel_old_pos_) / dt;
   const double back_right_wheel_vel = (back_right_pos - back_right_wheel_old_pos_) / dt;
 
   // Compute average velocities for each side (left & right)
   const double left_wheel_vel = (front_left_wheel_vel + back_left_wheel_vel) / 2.0;
   const double right_wheel_vel = (front_right_wheel_vel + back_right_wheel_vel) / 2.0;
 
   // Save current wheel positions for next update
   front_left_wheel_old_pos_ = front_left_pos;
   front_right_wheel_old_pos_ = front_right_pos;
   back_left_wheel_old_pos_ = back_left_pos;
   back_right_wheel_old_pos_ = back_right_pos;
 
   // Update odometry with new averaged wheel velocities
   updateFromVelocity(front_left_wheel_vel, back_left_wheel_vel,
                      front_right_wheel_vel, back_right_wheel_vel, time);
 
   return true;
 }
 
 /**
  * @brief Update odometry using wheel velocity measurements
  * @param left_wheel_vel Left-side wheel velocity (rad/s)
  * @param right_wheel_vel Right-side wheel velocity (rad/s)
  * @param time Current ROS time
  * @return true if update successful
  */
 bool Odometry::updateFromVelocity(
  double front_left_vel, double back_left_vel,
  double front_right_vel, double back_right_vel,
  const rclcpp::Time & time)
 {
   const double dt = time.seconds() - timestamp_.seconds();

   const double left_wheel_vel = (front_left_vel + back_left_vel) / 2.0;
   const double right_wheel_vel = (front_right_vel + back_right_vel) / 2.0;
 
   // Compute linear and angular velocities using differential drive kinematics
   const double linear_x = wheel_radius_ * (left_wheel_vel + right_wheel_vel) / 2.0;
   const double angular = wheel_radius_ * (right_wheel_vel - left_wheel_vel) / wheel_separation_;
 
   // Update pose using exact integration
   integrateExact(linear_x * dt, angular * dt);
 
   timestamp_ = time;
 
   // Update rolling averages for velocity smoothing
   linear_x_accumulator_.accumulate(linear_x);
   angular_accumulator_.accumulate(angular);
 
   linear_x_ = linear_x_accumulator_.getRollingMean();
   angular_ = angular_accumulator_.getRollingMean();
 
   return true;
 }
 
 /**
  * @brief Update odometry using direct velocity commands (open loop)
  * @param linear_x Forward/backward velocity (m/s)
  * @param angular Angular velocity (rad/s)
  * @param time Current ROS time
  */
 void Odometry::updateOpenLoop(
   double linear_x, double angular,
   const rclcpp::Time & time)
 {
   linear_x_ = linear_x;
   angular_ = angular;
 
   const double dt = time.seconds() - timestamp_.seconds();
   timestamp_ = time;
 
   integrateExact(linear_x * dt, angular * dt);
 }
 
 /**
  * @brief Reset odometry to initial state (0,0,0)
  */
 void Odometry::resetOdometry()
 {
   x_ = 0.0;
   y_ = 0.0;
   heading_ = 0.0;
 }
 
 /**
  * @brief Set robot's physical parameters for odometry calculations
  * @param wheel_separation Distance between left and right wheels (meters)
  * @param wheel_radius Radius of wheels (meters)
  */
 void Odometry::setWheelParams(double wheel_separation, double wheel_radius)
 {
   wheel_separation_ = wheel_separation;
   wheel_radius_ = wheel_radius;
 }
 
 /**
  * @brief Reset velocity rolling mean accumulators
  */
 void Odometry::resetAccumulators()
 {
   linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
   angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
 }
 
 }  // namespace differential_drive_controller 