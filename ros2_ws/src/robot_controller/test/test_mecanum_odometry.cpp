#include <gtest/gtest.h>

#include "mecanum_controller/mecanum_odometry.hpp"

TEST(MecanumOdometryTest, UsesRosBodyAxisSigns)
{
  constexpr double wheel_radius_m = 0.075;
  constexpr double rotation_radius_m = (0.355 + 0.353) / 2.0;

  const auto forward = mecanum_odometry::calculate_body_velocity(
    {10.0, 10.0, -10.0, -10.0}, wheel_radius_m, rotation_radius_m);
  EXPECT_NEAR(forward.x_m_s, 0.75, 1e-9);
  EXPECT_NEAR(forward.y_m_s, 0.0, 1e-9);
  EXPECT_NEAR(forward.yaw_rad_s, 0.0, 1e-9);

  const auto left = mecanum_odometry::calculate_body_velocity(
    {-10.0, 10.0, -10.0, 10.0}, wheel_radius_m, rotation_radius_m);
  EXPECT_NEAR(left.x_m_s, 0.0, 1e-9);
  EXPECT_NEAR(left.y_m_s, 0.75, 1e-9);
  EXPECT_NEAR(left.yaw_rad_s, 0.0, 1e-9);

  const auto counter_clockwise = mecanum_odometry::calculate_body_velocity(
    {-10.0, -10.0, -10.0, -10.0}, wheel_radius_m, rotation_radius_m);
  EXPECT_NEAR(counter_clockwise.x_m_s, 0.0, 1e-9);
  EXPECT_NEAR(counter_clockwise.y_m_s, 0.0, 1e-9);
  EXPECT_GT(counter_clockwise.yaw_rad_s, 0.0);
}

// 旧 calculate_covariance_multiplier (後方互換: MAX 集約) のテスト
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

// 軸別 calculate_covariance_multipliers: X だけスリップしても Y/Yaw は影響を受けない
TEST(MecanumOdometryTest, PerAxisCovarianceIsIndependent)
{
  // X のみ閾値2倍超え (slip_level=1.0 → maximum_multiplier)
  const mecanum_odometry::BodyVelocity x_slip{2.0, 0.0, 0.0};
  const auto s_x = mecanum_odometry::calculate_covariance_multipliers(x_slip, 1.0, 1.0, 2.0, 10.0);
  EXPECT_DOUBLE_EQ(s_x.x, 10.0);   // X はフル膨張
  EXPECT_DOUBLE_EQ(s_x.y, 1.0);    // Y は影響なし
  EXPECT_DOUBLE_EQ(s_x.yaw, 1.0);  // Yaw は影響なし

  // Yaw のみ閾値2倍超え
  const mecanum_odometry::BodyVelocity yaw_slip{0.0, 0.0, 4.0};
  const auto s_yaw = mecanum_odometry::calculate_covariance_multipliers(
    yaw_slip, 1.0, 1.0, 2.0, 10.0);
  EXPECT_DOUBLE_EQ(s_yaw.x, 1.0);    // X は影響なし
  EXPECT_DOUBLE_EQ(s_yaw.y, 1.0);    // Y は影響なし
  EXPECT_DOUBLE_EQ(s_yaw.yaw, 10.0); // Yaw はフル膨張

  // 全軸が閾値以下 → 全て 1.0
  const mecanum_odometry::BodyVelocity no_slip{0.3, 0.3, 0.5};
  const auto s_none = mecanum_odometry::calculate_covariance_multipliers(
    no_slip, 1.0, 1.0, 2.0, 10.0);
  EXPECT_DOUBLE_EQ(s_none.x, 1.0);
  EXPECT_DOUBLE_EQ(s_none.y, 1.0);
  EXPECT_DOUBLE_EQ(s_none.yaw, 1.0);
}

// 線形補間の確認 (閾値の1.5倍 → slip_level=0.5 → 中間値)
TEST(MecanumOdometryTest, PerAxisCovarianceInterpolatesLinearly)
{
  // |accel_x| = 1.5, threshold = 1.0 → ratio=1.5 → slip_level=0.5 → 1.0 + 0.5*(10-1) = 5.5
  const mecanum_odometry::BodyVelocity mid_slip{1.5, 0.0, 0.0};
  const auto s = mecanum_odometry::calculate_covariance_multipliers(mid_slip, 1.0, 1.0, 2.0, 10.0);
  EXPECT_DOUBLE_EQ(s.x, 5.5);
  EXPECT_DOUBLE_EQ(s.y, 1.0);
  EXPECT_DOUBLE_EQ(s.yaw, 1.0);
}

// 車輪加速度 vs IMU実測加速度の不一致検知テスト
TEST(MecanumOdometryTest, DetectsSlipFromImuDiscrepancy)
{
  // 車輪は 2.5 m/s^2 で空転、IMU は 0.5 m/s^2 のみ検知 (差分 2.0 m/s^2 > 閾値 1.0m/s^2)
  const mecanum_odometry::BodyVelocity wheel_accel{2.5, 0.0, 0.0};
  const mecanum_odometry::BodyVelocity imu_accel{0.5, 0.0, 0.0};

  const auto s = mecanum_odometry::calculate_imu_discrepancy_multipliers(
    wheel_accel, imu_accel, 1.0, 1.0, 2.0, 10.0);
  EXPECT_DOUBLE_EQ(s.x, 10.0); // スリップ検知で最大膨張
  EXPECT_DOUBLE_EQ(s.y, 1.0);  // Y は影響なし
  EXPECT_DOUBLE_EQ(s.yaw, 1.0);
}

// メディアンフィルタのトゲ除去＆中央値出力テスト
TEST(MecanumOdometryTest, MedianFilterRejectsSpikeNoise)
{
  mecanum_odometry::MedianFilter<3> filter;

  // 正常な値: 1.0, 1.1
  EXPECT_DOUBLE_EQ(filter.update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(filter.update(1.1), 1.05); // 2個の平均/中央値

  // 突然の巨大スパイクノイズ (100.0) が混入
  // バッファ: [1.0, 1.1, 100.0] -> 中央値は 1.1 (100.0を完全に無視！)
  EXPECT_DOUBLE_EQ(filter.update(100.0), 1.1);

  // 次の正常な値 (1.2)
  // バッファ: [1.2, 1.1, 100.0] -> ソート後 [1.1, 1.2, 100.0] -> 中央値は 1.2
  EXPECT_DOUBLE_EQ(filter.update(1.2), 1.2);
}
