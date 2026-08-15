#ifndef MECANUM_CONTROLLER__MECANUM_ODOMETRY_HPP_
#define MECANUM_CONTROLLER__MECANUM_ODOMETRY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace mecanum_odometry
{
enum WheelIndex : std::size_t
{
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3,
  WHEEL_COUNT = 4
};

struct BodyVelocity
{
  double x_m_s{0.0};
  double y_m_s{0.0};
  double yaw_rad_s{0.0};
};

inline BodyVelocity calculate_body_velocity(
  const std::array<double, WHEEL_COUNT> & wheel_velocity_rad_s,
  const double wheel_radius_m, const double rotation_radius_m)
{
  // Motor feedback polarity matching EduLite hardware
  const double front_left_m_s = wheel_velocity_rad_s[FRONT_LEFT] * wheel_radius_m;
  const double front_right_m_s = -wheel_velocity_rad_s[FRONT_RIGHT] * wheel_radius_m;
  const double rear_left_m_s = wheel_velocity_rad_s[REAR_LEFT] * wheel_radius_m;
  const double rear_right_m_s = -wheel_velocity_rad_s[REAR_RIGHT] * wheel_radius_m;

  BodyVelocity velocity;
  velocity.x_m_s =
    -(front_left_m_s + front_right_m_s + rear_left_m_s + rear_right_m_s) / 4.0;
  velocity.y_m_s =
    (front_left_m_s - front_right_m_s - rear_left_m_s + rear_right_m_s) / 4.0;
  velocity.yaw_rad_s =
    (-front_left_m_s + front_right_m_s - rear_left_m_s + rear_right_m_s) /
    (4.0 * rotation_radius_m);
  return velocity;
}

inline double calculate_covariance_multiplier(
  const BodyVelocity & acceleration, const double x_threshold_m_s2,
  const double y_threshold_m_s2, const double yaw_threshold_rad_s2,
  const double maximum_multiplier)
{
  const double acceleration_ratio = std::max(
    {
      std::abs(acceleration.x_m_s) / x_threshold_m_s2,
      std::abs(acceleration.y_m_s) / y_threshold_m_s2,
      std::abs(acceleration.yaw_rad_s) / yaw_threshold_rad_s2});

  // Keep the base covariance below the threshold and reach the maximum at 2x.
  const double slip_level = std::clamp(acceleration_ratio - 1.0, 0.0, 1.0);
  return 1.0 + slip_level * (maximum_multiplier - 1.0);
}
}  // namespace mecanum_odometry

#endif  // MECANUM_CONTROLLER__MECANUM_ODOMETRY_HPP_
