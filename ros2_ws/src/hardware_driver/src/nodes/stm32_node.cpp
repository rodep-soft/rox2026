/* ROS 2とSTM32間でCANフレームを送受信するノード。 */
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "stm32_driver/stm32_protocol.hpp"

namespace stm32_driver
{
class Stm32Node : public rclcpp::Node
{
public:
  explicit Stm32Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stm32_driver_node", options),
    imu_monitor_started_at_(std::chrono::steady_clock::now()),
    last_heartbeat_from_stm32_(std::chrono::steady_clock::now())
  {
    const auto can_tx_topic =
      declare_parameter<std::string>("can_tx_topic", "/socketcan_bridge/tx");
    const auto can_rx_topic =
      declare_parameter<std::string>("can_rx_topic", "/socketcan_bridge/rx");
    const auto limit_switches_topic = declare_parameter<std::string>(
      "limit_switches_topic", "/hardware/limit_switches");
    const auto led_command_topic = declare_parameter<std::string>(
      "led_command_topic", "/hardware/led_cmd");
    const auto imu_topic =
      declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto imu_frame_id =
      declare_parameter<std::string>("imu_frame_id", "imu_link");
    const auto keep_alive_period_ms =
      declare_parameter<int64_t>("keep_alive_period_ms", 100);
    const auto timeout_ms = declare_parameter<int64_t>("timeout_ms", 500);
    const auto imu_component_timeout_ms =
      declare_parameter<int64_t>("imu_component_timeout_ms", 100);
    const auto imu_reception_timeout_ms =
      declare_parameter<int64_t>("imu_reception_timeout_ms", 500);

    heartbeat_timeout_ = std::chrono::milliseconds(timeout_ms);
    imu_component_timeout_ =
      std::chrono::milliseconds(imu_component_timeout_ms);
    imu_reception_timeout_ =
      std::chrono::milliseconds(imu_reception_timeout_ms);
    initialize_imu_message(imu_frame_id);

    const auto can_qos_pub =
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
    // Keep enough recent CAN frames to absorb short executor scheduling gaps
    // without building a large stale-data backlog.
    const auto can_qos_sub = rclcpp::SensorDataQoS().keep_last(50);

    rclcpp::SubscriptionOptions can_sub_options;
    can_sub_options.content_filter_options.filter_expression =
      "id = %0 OR id = %1 OR id = %2 OR id = %3 OR id = %4";
    can_sub_options.content_filter_options.expression_parameters = {
      std::to_string(protocol::HEARTBEAT_FROM_STM),
      std::to_string(protocol::LIMIT_SWITCH_STATE),
      std::to_string(protocol::QUATERNION),
      std::to_string(protocol::ANGULAR_VELOCITY),
      std::to_string(protocol::LINEAR_ACCELERATION)};

    can_pub_ =
      create_publisher<can_msgs::msg::Frame>(can_tx_topic, can_qos_pub);
    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      can_rx_topic, can_qos_sub,
      std::bind(&Stm32Node::can_callback, this, std::placeholders::_1),
      can_sub_options);

    led_cmd_sub_ = create_subscription<std_msgs::msg::UInt64>(
      led_command_topic, 10,
      std::bind(&Stm32Node::led_callback, this, std::placeholders::_1));
    limit_sw_pub_ =
      create_publisher<std_msgs::msg::UInt8>(limit_switches_topic, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_topic,
      rclcpp::SensorDataQoS());

    alive_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(keep_alive_period_ms),
      std::bind(&Stm32Node::alive_timer_callback, this));

    RCLCPP_INFO(get_logger(), "stm32_driver_node started");
  }

private:
  /// @brief Content Filterを通過したCANフレームをID別に処理する
  void can_callback(const can_msgs::msg::Frame::SharedPtr frame)
  {
    if (!protocol::is_standard_data_frame(*frame)) {
      return;
    }

    switch (frame->id) {
      case protocol::HEARTBEAT_FROM_STM: {
          if (!protocol::is_heartbeat_response(*frame)) {
            break;
          }
          last_heartbeat_from_stm32_ = std::chrono::steady_clock::now();
          heartbeat_received_ = true;
          if (heartbeat_timed_out_) {
            RCLCPP_INFO(get_logger(), "CAN connection to STM32 has recovered");
            heartbeat_timed_out_ = false;
          }
          break;
        }

      case protocol::QUATERNION: {
          int16_t x = 0;
          int16_t y = 0;
          int16_t z = 0;
          int16_t w = 0;
          if (protocol::decode_quaternion(*frame, x, y, z, w) &&
            update_orientation(x, y, z, w))
          {
            record_imu_reception();
            publish_imu();
          }
          break;
        }

      case protocol::ANGULAR_VELOCITY: {
          int16_t x = 0;
          int16_t y = 0;
          int16_t z = 0;
          if (protocol::decode_angular_velocity(*frame, x, y, z)) {
            imu_.angular_velocity.x =
              static_cast<double>(x) * protocol::ANGULAR_VELOCITY_SCALE_INV;
            imu_.angular_velocity.y =
              static_cast<double>(y) * protocol::ANGULAR_VELOCITY_SCALE_INV;
            imu_.angular_velocity.z =
              static_cast<double>(z) * protocol::ANGULAR_VELOCITY_SCALE_INV;
            angular_velocity_received_ = true;
            last_angular_velocity_time_ = std::chrono::steady_clock::now();
          }
          break;
        }

      case protocol::LINEAR_ACCELERATION: {
          int16_t x = 0;
          int16_t y = 0;
          int16_t z = 0;
          if (protocol::decode_linear_acceleration(*frame, x, y, z)) {
            imu_.linear_acceleration.x =
              static_cast<double>(x) * protocol::LINEAR_ACCELERATION_SCALE_INV;
            imu_.linear_acceleration.y =
              static_cast<double>(y) * protocol::LINEAR_ACCELERATION_SCALE_INV;
            imu_.linear_acceleration.z =
              static_cast<double>(z) * protocol::LINEAR_ACCELERATION_SCALE_INV;
            linear_acceleration_received_ = true;
            last_linear_acceleration_time_ = std::chrono::steady_clock::now();
          }
          break;
        }

      case protocol::LIMIT_SWITCH_STATE: {
          uint8_t limit_state = 0;
          if (protocol::decode_limit_switch(*frame, limit_state)) {
            std_msgs::msg::UInt8 output;
            output.data = limit_state;
            limit_sw_pub_->publish(output);
          }
          break;
        }

      default:
        break;
    }
  }

  /// @brief スケールを掛けて正規化したQuaternionをIMUメッセージへ反映する
  /// @return 有効なQuaternionならtrue
  bool update_orientation(int16_t x, int16_t y, int16_t z, int16_t w)
  {
    const auto qx = static_cast<double>(x) * protocol::QUATERNION_SCALE_INV;
    const auto qy = static_cast<double>(y) * protocol::QUATERNION_SCALE_INV;
    const auto qz = static_cast<double>(z) * protocol::QUATERNION_SCALE_INV;
    const auto qw = static_cast<double>(w) * protocol::QUATERNION_SCALE_INV;
    const auto norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

    if (!std::isfinite(norm) || norm <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "received an invalid quaternion");
      return false;
    }

    const auto inverse_norm = 1.0 / norm;
    imu_.orientation.x = qx * inverse_norm;
    imu_.orientation.y = qy * inverse_norm;
    imu_.orientation.z = qz * inverse_norm;
    imu_.orientation.w = qw * inverse_norm;
    return true;
  }

  /// @brief 実行中に変化しないIMUフィールドを初期化する
  void initialize_imu_message(const std::string & frame_id)
  {
    imu_.header.frame_id = frame_id;

    // --- sensor_msgs/Imu covariance の規約 ---
    // 各 covariance は 3x3 行列 (row-major, 9要素)。
    //   [0] = -1.0 → このフィールド全体が無効 (EKFが無視する)
    //   それ以外   → 対角成分 [0], [4], [8] に分散値を設定
    //
    // BNO055 NDOF モード (STM32経由) の実測ノイズ特性:
    //   orientation (quaternion→euler):
    //     roll/pitch ≈ 0.5 deg RMS → σ² ≈ (0.5*π/180)² ≈ 7.6e-5 rad²
    //     yaw        ≈ 1.0 deg RMS → σ² ≈ (1.0*π/180)² ≈ 3.0e-4 rad²
    //     (磁気干渉環境では yaw をさらに大きく見積もる: 3.0e-3 程度)
    //   angular_velocity (ジャイロ):
    //     ノイズ密度 ≈ 0.014 deg/s/√Hz @ 100Hz → σ² ≈ (0.014*π/180)² * 100 ≈
    //     6e-7 rad²/s²
    //   linear_acceleration:
    //     ノイズ密度 ≈ 150 μg/√Hz @ 100Hz → σ² ≈ (150e-6 * 9.8)² * 100 ≈ 2e-6
    //     m²/s⁴

    // BNO055 NDOFモードの実測ノイズ特性に基づく分散値。
    imu_.orientation_covariance[0] = 7.6e-5;  // roll  σ² [rad²]
    imu_.orientation_covariance[4] = 7.6e-5;  // pitch σ² [rad²]
    imu_.orientation_covariance[8] =
      3.0e-4;    // yaw   σ² [rad²] (磁気コンパス由来)
    imu_.angular_velocity_covariance[0] = 6.0e-7;     // wx σ² [rad²/s²]
    imu_.angular_velocity_covariance[4] = 6.0e-7;     // wy σ²
    imu_.angular_velocity_covariance[8] = 6.0e-7;     // wz σ²
    imu_.linear_acceleration_covariance[0] = 2.0e-6;  // ax σ² [m²/s⁴]
    imu_.linear_acceleration_covariance[4] = 2.0e-6;  // ay σ²
    imu_.linear_acceleration_covariance[8] = 2.0e-6;  // az σ²
  }

  /// @brief Quaternion受信時に最新の角速度・加速度と合わせてIMUを送信する
  void publish_imu()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const bool angular_velocity_valid =
      angular_velocity_received_ &&
      current_time - last_angular_velocity_time_ <= imu_component_timeout_;
    const bool linear_acceleration_valid =
      linear_acceleration_received_ &&
      current_time - last_linear_acceleration_time_ <= imu_component_timeout_;

    imu_.angular_velocity_covariance[0] =
      angular_velocity_valid ? 6.0e-7 : -1.0;
    imu_.linear_acceleration_covariance[0] =
      linear_acceleration_valid ? 2.0e-6 : -1.0;
    imu_.header.stamp = now();
    imu_pub_->publish(imu_);
  }

  /// @brief 有効なIMU信号の初回受信とタイムアウトからの復旧を記録する
  void record_imu_reception()
  {
    last_imu_reception_time_ = std::chrono::steady_clock::now();

    if (!imu_received_) {
      RCLCPP_INFO(get_logger(), "Valid IMU signal received");
      imu_received_ = true;
    } else if (imu_reception_timed_out_) {
      RCLCPP_INFO(get_logger(), "IMU signal reception has recovered");
    }

    imu_reception_timed_out_ = false;
  }

  /// @brief IMU信号が設定時間以上届いていない場合に一度だけ警告する
  void check_imu_reception()
  {
    // CAN疎通を確認できない場合は、IMUではなくCAN接続側の異常として扱う。
    if (!heartbeat_received_ || heartbeat_timed_out_) {
      return;
    }

    if (imu_reception_timed_out_) {
      return;
    }

    const auto current_time = std::chrono::steady_clock::now();
    const auto reference_time =
      imu_received_ ? last_imu_reception_time_ : imu_monitor_started_at_;
    if (current_time - reference_time <= imu_reception_timeout_) {
      return;
    }

    if (imu_received_) {
      RCLCPP_WARN(
        get_logger(), "CAN connection to STM32 is alive, but IMU signal reception timed out");
    } else {
      RCLCPP_WARN(
        get_logger(), "CAN connection to STM32 is alive, but no valid IMU signal was received");
    }
    imu_reception_timed_out_ = true;
  }

  /// @brief LEDコマンドをSTM32へ送信する
  /// @param msg LEDコマンド
  void led_callback(const std_msgs::msg::UInt64::SharedPtr msg)
  {
    can_pub_->publish(protocol::make_led_frame(msg->data));
  }

  /// @brief STM32へ生存報告を送り、heartbeatのタイムアウトを確認する
  void alive_timer_callback()
  {
    can_pub_->publish(protocol::make_alive_frame());

    const auto elapsed =
      std::chrono::steady_clock::now() - last_heartbeat_from_stm32_;
    if (elapsed > heartbeat_timeout_ && !heartbeat_timed_out_) {
      RCLCPP_WARN(get_logger(), "CAN connection to STM32 timed out");
      heartbeat_timed_out_ = true;
    }

    check_imu_reception();
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr led_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limit_sw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr alive_timer_;

  sensor_msgs::msg::Imu imu_;
  bool angular_velocity_received_{false};
  bool linear_acceleration_received_{false};
  std::chrono::steady_clock::time_point last_angular_velocity_time_{};
  std::chrono::steady_clock::time_point last_linear_acceleration_time_{};
  std::chrono::milliseconds imu_component_timeout_{0};
  std::chrono::steady_clock::time_point last_imu_reception_time_{};
  std::chrono::steady_clock::time_point imu_monitor_started_at_;
  std::chrono::milliseconds imu_reception_timeout_{0};
  bool imu_received_{false};
  bool imu_reception_timed_out_{false};
  std::chrono::steady_clock::time_point last_heartbeat_from_stm32_;
  std::chrono::milliseconds heartbeat_timeout_{0};
  bool heartbeat_received_{false};
  bool heartbeat_timed_out_{false};
};

}  // namespace stm32_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stm32_driver::Stm32Node>());
  rclcpp::shutdown();
  return 0;
}
