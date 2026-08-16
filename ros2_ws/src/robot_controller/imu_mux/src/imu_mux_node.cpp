#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace robot_controller
{

// クォータニオンからYaw角を抽出
static double quat_to_yaw(const sensor_msgs::msg::Imu & msg)
{
  const auto & q = msg.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Yaw角のみ上書きしたクォータニオンを返す（roll/pitchは保持）
static sensor_msgs::msg::Imu apply_yaw_correction(
  const sensor_msgs::msg::Imu & msg, double corrected_yaw)
{
  auto out = msg;
  // 2Dモード用: roll=0, pitch=0 としてYawのみ設定
  out.orientation.x = 0.0;
  out.orientation.y = 0.0;
  out.orientation.z = std::sin(corrected_yaw / 2.0);
  out.orientation.w = std::cos(corrected_yaw / 2.0);
  return out;
}

class ImuMuxNode : public rclcpp::Node
{
public:
  explicit ImuMuxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("imu_mux_node", options)
  {
    primary_topic_ = this->declare_parameter<std::string>("primary_imu_topic", "/bno055/imu");
    secondary_topic_ = this->declare_parameter<std::string>("secondary_imu_topic", "/stm32/imu");
    output_topic_ = this->declare_parameter<std::string>("output_imu_topic", "/imu/data");
    timeout_ms_ = this->declare_parameter<int>("timeout_ms", 50);
    // ±180°フリップ検出しきい値 [rad]  デフォルト 150° ≈ 2.62 rad
    flip_threshold_ = this->declare_parameter<double>("yaw_flip_threshold", 2.62);

    out_pub_ =
      this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());

    primary_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      primary_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!is_valid_imu(*msg)) {return;}
        last_primary_time_ = this->now();
        if (active_source_ != Source::PRIMARY) {
          RCLCPP_INFO(
            this->get_logger(), "[IMU MUX] Switched to PRIMARY source: %s",
            primary_topic_.c_str());
          active_source_ = Source::PRIMARY;
        }
        out_pub_->publish(correct_yaw_flip(*msg, primary_prev_yaw_, primary_yaw_offset_));
      });

    secondary_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      secondary_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!is_valid_imu(*msg)) {return;}
        const auto now_time = this->now();
        const bool primary_healthy = (last_primary_time_.nanoseconds() != 0) &&
        ((now_time - last_primary_time_).nanoseconds() <= timeout_ms_ * 1000000LL);

        if (!primary_healthy) {
          if (active_source_ != Source::SECONDARY) {
            RCLCPP_WARN(
              this->get_logger(),
              "[IMU MUX] Primary timed out! Fallback to SECONDARY source: %s",
              secondary_topic_.c_str());
            active_source_ = Source::SECONDARY;
          }
          out_pub_->publish(correct_yaw_flip(*msg, secondary_prev_yaw_, secondary_yaw_offset_));
        }
      });

    RCLCPP_INFO(
      this->get_logger(),
      "IMU Mux online: Primary=%s Secondary=%s -> Output=%s (Timeout=%d ms, FlipThresh=%.2f rad)",
      primary_topic_.c_str(), secondary_topic_.c_str(), output_topic_.c_str(),
      timeout_ms_, flip_threshold_);
  }

private:
  // ── Yawフリップ検出・補正 ────────────────────────────────────────────
  // STM32がリセットして急に±180°ジャンプしても連続した角度に補正する
  sensor_msgs::msg::Imu correct_yaw_flip(
    const sensor_msgs::msg::Imu & msg,
    double & prev_yaw,   // 前回のraw yaw（ソースごとに独立管理）
    double & offset)     // 累積オフセット（ソースごとに独立管理）
  {
    const double raw_yaw = quat_to_yaw(msg);

    // 初回: 基準設定のみ
    if (!std::isfinite(prev_yaw)) {
      prev_yaw = raw_yaw;
      offset = 0.0;
      return msg;
    }

    // 前回からの変化量 (-π〜+π に正規化)
    double delta = std::remainder(raw_yaw - prev_yaw, 2.0 * M_PI);

    // フリップ検出: 変化量がしきい値超え → ±2π 補正
    if (std::abs(delta) > flip_threshold_) {
      const double correction = (delta > 0) ? -2.0 * M_PI : 2.0 * M_PI;
      offset += correction;
      RCLCPP_WARN(
        get_logger(),
        "[IMU MUX] Yaw flip detected! delta=%.2f rad -> correction offset=%.2f rad applied.",
        delta, offset);
    }

    prev_yaw = raw_yaw;
    const double corrected_yaw = raw_yaw + offset;

    return apply_yaw_correction(msg, corrected_yaw);
  }

  static bool is_valid_imu(const sensor_msgs::msg::Imu & msg)
  {
    const auto & q = msg.orientation;
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w) || !std::isfinite(msg.angular_velocity.z))
    {
      return false;
    }
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return norm_sq > 1e-6;
  }

  enum class Source { NONE, PRIMARY, SECONDARY };

  std::string primary_topic_;
  std::string secondary_topic_;
  std::string output_topic_;
  int timeout_ms_{50};
  double flip_threshold_{2.62};

  // ソースごとの Yaw 追跡変数
  double primary_prev_yaw_{std::numeric_limits<double>::quiet_NaN()};
  double primary_yaw_offset_{0.0};
  double secondary_prev_yaw_{std::numeric_limits<double>::quiet_NaN()};
  double secondary_yaw_offset_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr out_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr primary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr secondary_sub_;

  rclcpp::Time last_primary_time_{0, 0, RCL_ROS_TIME};
  Source active_source_{Source::NONE};
};

}  // namespace robot_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::ImuMuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
