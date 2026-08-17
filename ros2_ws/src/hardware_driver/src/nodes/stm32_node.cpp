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
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "stm32_driver/stm32_protocol.hpp"

namespace stm32_driver
{
namespace
{

constexpr char CAN_PUB_TOPIC[] = "/socketcan_bridge/tx";
constexpr char CAN_SUB_TOPIC[] = "/socketcan_bridge/rx";


}  // namespace

class Stm32Node : public rclcpp::Node
{
public:
  explicit Stm32Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stm32_driver_node", options),
    last_heartbeat_from_stm32_(std::chrono::steady_clock::now())
  {
    const auto limit_switches_topic = declare_parameter<std::string>(
      "limit_switches_topic",
      "/hardware/limit_switches");
    const auto led_command_topic = declare_parameter<std::string>(
      "led_command_topic",
      "/hardware/led_cmd");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    const auto keep_alive_period_ms = declare_parameter<int64_t>("keep_alive_period_ms", 100);
    const auto timeout_ms = declare_parameter<int64_t>("timeout_ms", 500);

    heartbeat_timeout_ = std::chrono::milliseconds(timeout_ms);

    auto can_qos_pub = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
    auto can_qos_sub = rclcpp::SensorDataQoS();

    rclcpp::SubscriptionOptions can_sub_options;
    can_sub_options.content_filter_options.filter_expression =
      "id = %0 OR id = %1 OR id = %2 OR id = %3 OR id = %4";
    can_sub_options.content_filter_options.expression_parameters = {
      std::to_string(protocol::HEARTBEAT_FROM_STM),
      std::to_string(protocol::LIMIT_SWITCH_STATE),
      std::to_string(protocol::QUATERNION),
      std::to_string(protocol::ANGULAR_VELOCITY),
      std::to_string(protocol::LINEAR_ACCELERATION)};

    can_pub_ = create_publisher<can_msgs::msg::Frame>(CAN_PUB_TOPIC, can_qos_pub);
    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      CAN_SUB_TOPIC,
      can_qos_sub,
      std::bind(&Stm32Node::can_callback, this, std::placeholders::_1),
      can_sub_options);

    led_cmd_sub_ = create_subscription<std_msgs::msg::UInt16>(
      led_command_topic,
      10,
      std::bind(&Stm32Node::led_callback, this, std::placeholders::_1));
    limit_sw_pub_ = create_publisher<std_msgs::msg::UInt8>(limit_switches_topic, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());

    alive_timer_ = create_wall_timer(
      std::chrono::milliseconds(keep_alive_period_ms),
      std::bind(&Stm32Node::alive_timer_callback, this));

    RCLCPP_INFO(get_logger(), "stm32_driver_node started");
  }

private:
  /// @brief CANフレームをIDに応じてデコードする
  /// @param frame 受信したCANフレーム
  void can_callback(const can_msgs::msg::Frame::SharedPtr frame)
  {
    if (!protocol::is_standard_data_frame(*frame)) {
      return;
    }

    if (protocol::is_heartbeat_response(*frame)) {
      last_heartbeat_from_stm32_ = std::chrono::steady_clock::now();
      if (heartbeat_timed_out_) {
        RCLCPP_INFO(this->get_logger(), "The STM32 heartbeat has been restored.");
        heartbeat_timed_out_ = false;
      }
      return;
    }

    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    int16_t w = 0;
    if (protocol::decode_quaternion(*frame, x, y, z, w)) {
      update_orientation(x, y, z, w);
      publish_imu();
      return;
    }

    if (protocol::decode_angular_velocity(*frame, x, y, z)) {
      imu_.angular_velocity.x = static_cast<double>(x) * protocol::ANGULAR_VELOCITY_SCALE_INV;
      imu_.angular_velocity.y = static_cast<double>(y) * protocol::ANGULAR_VELOCITY_SCALE_INV;
      imu_.angular_velocity.z = static_cast<double>(z) * protocol::ANGULAR_VELOCITY_SCALE_INV;
      angular_velocity_received_ = true;
      publish_imu();
      return;
    }

    if (protocol::decode_linear_acceleration(*frame, x, y, z)) {
      imu_.linear_acceleration.x =
        static_cast<double>(x) * protocol::LINEAR_ACCELERATION_SCALE_INV;
      imu_.linear_acceleration.y =
        static_cast<double>(y) * protocol::LINEAR_ACCELERATION_SCALE_INV;
      imu_.linear_acceleration.z =
        static_cast<double>(z) * protocol::LINEAR_ACCELERATION_SCALE_INV;
      linear_acceleration_received_ = true;
      publish_imu();
      return;
    }

    uint8_t limit_state = 0;
    if (protocol::decode_limit_switch(*frame, limit_state)) {
      std_msgs::msg::UInt8 output;
      output.data = limit_state;
      limit_sw_pub_->publish(output);
    }
  }

  /// @brief スケールを掛けたクォータニオンを最新のIMUメッセージへ反映する
  /// @param x X成分
  /// @param y Y成分
  /// @param z Z成分
  /// @param w W成分
  void update_orientation(int16_t x, int16_t y, int16_t z, int16_t w)
  {
    const auto qx = static_cast<double>(x) * protocol::QUATERNION_SCALE_INV;
    const auto qy = static_cast<double>(y) * protocol::QUATERNION_SCALE_INV;
    const auto qz = static_cast<double>(z) * protocol::QUATERNION_SCALE_INV;
    const auto qw = static_cast<double>(w) * protocol::QUATERNION_SCALE_INV;
    const auto norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

    if (!std::isfinite(norm) || norm <= 0.0) {
      RCLCPP_WARN(get_logger(), "received an invalid quaternion");
      return;
    }

    imu_.orientation.x = qx / norm;
    imu_.orientation.y = qy / norm;
    imu_.orientation.z = qz / norm;
    imu_.orientation.w = qw / norm;
    orientation_received_ = true;
  }

  /// @brief 受信済みの最新値をIMUメッセージとして送信する
  void publish_imu()
  {
    // 有効な姿勢が得られるまでは不正なゼロクォータニオンをpublishしない。
    if (!orientation_received_) {
      return;
    }

    imu_.header.stamp = now();
    imu_.header.frame_id = imu_frame_id_;

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
    //     ノイズ密度 ≈ 0.014 deg/s/√Hz @ 100Hz → σ² ≈ (0.014*π/180)² * 100 ≈ 6e-7 rad²/s²
    //   linear_acceleration:
    //     ノイズ密度 ≈ 150 μg/√Hz @ 100Hz → σ² ≈ (150e-6 * 9.8)² * 100 ≈ 2e-6 m²/s⁴

    // orientation covariance (roll=x, pitch=y, yaw=z)
    if (orientation_received_) {
      imu_.orientation_covariance[0] = 7.6e-5;  // roll  σ² [rad²]
      imu_.orientation_covariance[4] = 7.6e-5;  // pitch σ² [rad²]
      imu_.orientation_covariance[8] = 3.0e-4;  // yaw   σ² [rad²] (磁気コンパス由来)
    } else {
      imu_.orientation_covariance[0] = -1.0;    // 無効
    }

    // angular_velocity covariance
    if (angular_velocity_received_) {
      imu_.angular_velocity_covariance[0] = 6.0e-7;  // wx σ² [rad²/s²]
      imu_.angular_velocity_covariance[4] = 6.0e-7;  // wy σ²
      imu_.angular_velocity_covariance[8] = 6.0e-7;  // wz σ²
    } else {
      imu_.angular_velocity_covariance[0] = -1.0;    // 無効
    }

    // linear_acceleration covariance
    if (linear_acceleration_received_) {
      imu_.linear_acceleration_covariance[0] = 2.0e-6;  // ax σ² [m²/s⁴]
      imu_.linear_acceleration_covariance[4] = 2.0e-6;  // ay σ²
      imu_.linear_acceleration_covariance[8] = 2.0e-6;  // az σ²
    } else {
      imu_.linear_acceleration_covariance[0] = -1.0;    // 無効
    }

    imu_pub_->publish(imu_);
  }


  /// @brief LEDコマンドをSTM32へ送信する
  /// @param msg LEDコマンド
  void led_callback(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    can_pub_->publish(protocol::make_led_frame(msg->data));
  }

  /// @brief STM32へ生存報告を送り、heartbeatのタイムアウトを確認する
  void alive_timer_callback()
  {
    can_pub_->publish(protocol::make_alive_frame());

    const auto elapsed = std::chrono::steady_clock::now() - last_heartbeat_from_stm32_;
    if (elapsed > heartbeat_timeout_ && !heartbeat_timed_out_) {
      RCLCPP_WARN(get_logger(), "STM32 heartbeat timed out");
      heartbeat_timed_out_ = true;
    }
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr led_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limit_sw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr alive_timer_;

  std::string imu_frame_id_;
  sensor_msgs::msg::Imu imu_;
  bool orientation_received_{false};
  bool angular_velocity_received_{false};
  bool linear_acceleration_received_{false};
  std::chrono::steady_clock::time_point last_heartbeat_from_stm32_;
  std::chrono::milliseconds heartbeat_timeout_{0};
  bool heartbeat_timed_out_{false};
};

}  // namespace stm32_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stm32_driver::Stm32Node>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
