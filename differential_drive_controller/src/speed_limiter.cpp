/**
 * @file speed_limiter.cpp
 * @brief Implementation of velocity, acceleration, and jerk limits for differential drive robots.
 *
 * This implements the speed limiting functionality for a differential drive robot, ensuring smooth motion by:
 * - Limiting maximum and minimum linear velocity
 * - Controlling acceleration to prevent sudden speed changes
 * - Managing jerk (rate of acceleration change) for smoother motion
 *
 * This speed limiter is used by creating two instances in the controller:
 * - One for linear velocity (forward/backward)
 * - One for angular velocity (rotation)
 *
 * @author Addison Sears-Collins (adapted for differential drive by Ako)
 * @date March 9, 2025
 */

 #include <algorithm>
 #include <stdexcept>
 #include "differential_drive_controller/speed_limiter.hpp"
 
 namespace differential_drive_controller
 {
 
 SpeedLimiter::SpeedLimiter(
   bool has_velocity_limits,
   bool has_acceleration_limits,
   bool has_jerk_limits,
   double min_velocity,
   double max_velocity,
   double min_acceleration,
   double max_acceleration,
   double min_jerk,
   double max_jerk)
 : has_velocity_limits_(has_velocity_limits),
   has_acceleration_limits_(has_acceleration_limits),
   has_jerk_limits_(has_jerk_limits),
   min_velocity_(min_velocity),
   max_velocity_(max_velocity),
   min_acceleration_(min_acceleration),
   max_acceleration_(max_acceleration),
   min_jerk_(min_jerk),
   max_jerk_(max_jerk)
 {
   if (has_velocity_limits_ && std::isnan(max_velocity_))
   {
     throw std::runtime_error("Cannot apply velocity limits if max_velocity is not specified");
   }
   if (has_acceleration_limits_ && std::isnan(max_acceleration_))
   {
     throw std::runtime_error("Cannot apply acceleration limits if max_acceleration is not specified");
   }
   if (has_jerk_limits_ && std::isnan(max_jerk_))
   {
     throw std::runtime_error("Cannot apply jerk limits if max_jerk is not specified");
   }
 }
 
 double SpeedLimiter::limit(double & v, double v0, double v1, double dt)
 {
   const double original_velocity = v;
   limit_jerk(v, v0, v1, dt);
   limit_acceleration(v, v0, dt);
   limit_velocity(v);
   return original_velocity != 0.0 ? v / original_velocity : 1.0;
 }
 
 double SpeedLimiter::limit_velocity(double & v)
 {
   const double original_velocity = v;
   if (has_velocity_limits_)
   {
     v = std::clamp(v, min_velocity_, max_velocity_);
   }
   return original_velocity != 0.0 ? v / original_velocity : 1.0;
 }
 
 double SpeedLimiter::limit_acceleration(double & v, double v0, double dt)
 {
   const double original_velocity = v;
   if (has_acceleration_limits_)
   {
     const double dv_min = min_acceleration_ * dt;
     const double dv_max = max_acceleration_ * dt;
     const double dv = std::clamp(v - v0, dv_min, dv_max);
     v = v0 + dv;
   }
   return original_velocity != 0.0 ? v / original_velocity : 1.0;
 }
 
 double SpeedLimiter::limit_jerk(double & v, double v0, double v1, double dt)
 {
   const double original_velocity = v;
   if (has_jerk_limits_)
   {
     const double dv = v - v0;
     const double dv0 = v0 - v1;
     const double dt2 = 2.0 * dt * dt;
     const double da_min = min_jerk_ * dt2;
     const double da_max = max_jerk_ * dt2;
     const double da = std::clamp(dv - dv0, da_min, da_max);
     v = v0 + dv0 + da;
   }
   return original_velocity != 0.0 ? v / original_velocity : 1.0;
 }
 
 }  // namespace differential_drive_controller 