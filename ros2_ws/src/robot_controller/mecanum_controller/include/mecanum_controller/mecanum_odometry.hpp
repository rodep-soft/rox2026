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

/// @brief 遅延ゼロでトゲ・外れ値スパイクノイズを除去する汎用メディアンフィルタ (奇数バッファ N=3 or 5)
template<std::size_t N = 3>
class MedianFilter
{
  static_assert(N % 2 == 1, "MedianFilter buffer size N must be odd (e.g. 3 or 5)");

public:
  constexpr MedianFilter() = default;

  double update(const double val)
  {
    buffer_[head_] = val;
    head_ = (head_ + 1) % N;
    if (count_ < N) {
      ++count_;
    }

    std::array<double, N> sorted = buffer_;
    const auto valid_end = sorted.begin() + count_;
    std::sort(sorted.begin(), valid_end);
    return sorted[count_ / 2];
  }

  void reset()
  {
    buffer_.fill(0.0);
    head_ = 0;
    count_ = 0;
  }

private:
  std::array<double, N> buffer_{};
  std::size_t head_{0};
  std::size_t count_{0};
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
  // Exact inverse transformation of mecanum_controller_node inverse kinematics:
  // wheel_vel[FL] = -(Vx - Vy + L*Wz) / R
  // wheel_vel[FR] =  (Vx + Vy - L*Wz) / R
  // wheel_vel[RL] = -(Vx + Vy + L*Wz) / R
  // wheel_vel[RR] =  (Vx - Vy - L*Wz) / R

  const double w_fl = wheel_velocity_rad_s[FRONT_LEFT];
  const double w_fr = wheel_velocity_rad_s[FRONT_RIGHT];
  const double w_rl = wheel_velocity_rad_s[REAR_LEFT];
  const double w_rr = wheel_velocity_rad_s[REAR_RIGHT];

  BodyVelocity velocity;
  velocity.x_m_s = (-w_fl + w_fr - w_rl + w_rr) * wheel_radius_m / 4.0;
  velocity.y_m_s = ( w_fl + w_fr - w_rl - w_rr) * wheel_radius_m / 4.0;
  velocity.yaw_rad_s = (-w_fl - w_fr - w_rl - w_rr) * wheel_radius_m / (4.0 * rotation_radius_m);
  return velocity;
}

/// @brief 各軸の共分散スケールファクタを格納する。
/// EKFへ送る twist covariance を軸ごとに独立してスケールするために使う。
struct AxisCovarianceScale
{
  double x{1.0};
  double y{1.0};
  double yaw{1.0};
};

/// @brief スリップ・急加速時の軸別共分散スケールファクタを計算する。
///
/// 以前の実装は max(x, y, yaw) の単一スカラーを返していたため、
/// X方向だけスリップしたときに yaw の共分散まで膨らむ「連帯責任」が発生していた。
/// 本関数は各軸を独立して評価し、それぞれのスリップレベルに応じたスカラーを返す。
///
/// - 各軸の ratio が閾値以下 → その軸は 1.0 (ベース共分散そのまま)
/// - 閾値〜2×閾値 → 1.0 〜 maximum_multiplier で線形補間
/// - 2×閾値以上  → maximum_multiplier (上限クランプ)
inline AxisCovarianceScale calculate_covariance_multipliers(
  const BodyVelocity & acceleration, const double x_threshold_m_s2,
  const double y_threshold_m_s2, const double yaw_threshold_rad_s2,
  const double maximum_multiplier)
{
  auto axis_scale = [maximum_multiplier](const double abs_accel, const double threshold) -> double {
      const double slip_level = std::clamp(std::abs(abs_accel) / threshold - 1.0, 0.0, 1.0);
      return 1.0 + slip_level * (maximum_multiplier - 1.0);
    };
  return {
    axis_scale(acceleration.x_m_s, x_threshold_m_s2),
    axis_scale(acceleration.y_m_s, y_threshold_m_s2),
    axis_scale(acceleration.yaw_rad_s, yaw_threshold_rad_s2)
  };
}

/// @brief 車輪加速度とIMU実測加速度の不一致度(差分)に基づいて共分散スケールを計算する。
/// 車輪が空転しているとき (wheel_accel >> imu_accel) に高感度でスリップを検知し共分散を膨張させる。
inline AxisCovarianceScale calculate_imu_discrepancy_multipliers(
  const BodyVelocity & wheel_accel, const BodyVelocity & imu_accel,
  const double x_threshold_m_s2, const double y_threshold_m_s2, const double yaw_threshold_rad_s2,
  const double maximum_multiplier)
{
  const BodyVelocity discrepancy{
    std::abs(wheel_accel.x_m_s - imu_accel.x_m_s),
    std::abs(wheel_accel.y_m_s - imu_accel.y_m_s),
    std::abs(wheel_accel.yaw_rad_s - imu_accel.yaw_rad_s)
  };
  return calculate_covariance_multipliers(
    discrepancy, x_threshold_m_s2, y_threshold_m_s2, yaw_threshold_rad_s2, maximum_multiplier);
}

/// @brief 後方互換用: 全軸を MAX 集約した単一スカラーを返す旧版。
/// テストからのみ参照される。
inline double calculate_covariance_multiplier(
  const BodyVelocity & acceleration, const double x_threshold_m_s2,
  const double y_threshold_m_s2, const double yaw_threshold_rad_s2,
  const double maximum_multiplier)
{
  const auto s = calculate_covariance_multipliers(
    acceleration, x_threshold_m_s2, y_threshold_m_s2, yaw_threshold_rad_s2, maximum_multiplier);
  return std::max({s.x, s.y, s.yaw});
}
}  // namespace mecanum_odometry

#endif  // MECANUM_CONTROLLER__MECANUM_ODOMETRY_HPP_
