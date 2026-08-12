#include <gtest/gtest.h>

#include "mecanum_controller/mecanum_odometry.hpp"

TEST(MecanumOdometryTest, UsesRosBodyAxisSigns)
{
  constexpr double wheel_radius_m = 0.075;
  constexpr double rotation_radius_m = (0.355 + 0.353) / 2.0;

  const auto forward = mecanum_odometry::calculate_body_velocity(
    {-10.0, 10.0, -10.0, 10.0}, wheel_radius_m, rotation_radius_m);
  EXPECT_NEAR(forward.x_m_s, 0.75, 1e-9);
  EXPECT_NEAR(forward.y_m_s, 0.0, 1e-9);
  EXPECT_NEAR(forward.yaw_rad_s, 0.0, 1e-9);

  const auto left = mecanum_odometry::calculate_body_velocity(
    {10.0, 10.0, -10.0, -10.0}, wheel_radius_m, rotation_radius_m);
  EXPECT_NEAR(left.x_m_s, 0.0, 1e-9);
  EXPECT_NEAR(left.y_m_s, 0.75, 1e-9);
  EXPECT_NEAR(left.yaw_rad_s, 0.0, 1e-9);

  const auto counter_clockwise = mecanum_odometry::calculate_body_velocity(
    {-10.0, -10.0, -10.0, -10.0}, wheel_radius_m, rotation_radius_m);
  EXPECT_NEAR(counter_clockwise.x_m_s, 0.0, 1e-9);
  EXPECT_NEAR(counter_clockwise.y_m_s, 0.0, 1e-9);
  EXPECT_GT(counter_clockwise.yaw_rad_s, 0.0);
}

TEST(MecanumOdometryTest, RaisesCovarianceAboveAccelerationThreshold)
{
  const mecanum_odometry::BodyVelocity below_threshold{0.5, 0.5, 1.0};
  EXPECT_DOUBLE_EQ(
    mecanum_odometry::calculate_covariance_multiplier(
      below_threshold, 1.0, 1.0, 2.0, 10.0), 1.0);

  const mecanum_odometry::BodyVelocity above_threshold{2.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(
    mecanum_odometry::calculate_covariance_multiplier(
      above_threshold, 1.0, 1.0, 2.0, 10.0), 10.0);
}
