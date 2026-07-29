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
  Stm32Node()
  : Node("stm32_driver_node"),
    last_heartbeat_from_stm32_(std::chrono::steady_clock::now())
  {
    const auto led_cmd_topic = declare_parameter<std::string>("led_cmd_topic", "/led/cmd");
    const auto limit_sw_topic = declare_parameter<std::string>("limit_sw_topic", "/limitsw");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    const auto keep_alive_period_ms = declare_parameter<int64_t>("keep_alive_period_ms", 100);
    const auto timeout_ms = declare_parameter<int64_t>("timeout_ms", 500);

    heartbeat_timeout_ = std::chrono::milliseconds(timeout_ms);

    auto can_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto can_qos_sub = rclcpp::SensorDataQoS();

    rclcpp::SubscriptionOptions can_sub_options;
    can_sub_options.content_filter_options.filter_expression =
      "id = %0 OR id = %1 OR id = %2";
    can_sub_options.content_filter_options.expression_parameters = {
      std::to_string(protocol::HEARTBEAT_FROM_STM),
      std::to_string(protocol::LIMIT_SWITCH_STATE),
      std::to_string(protocol::QUATERNION)};

    can_pub_ = create_publisher<can_msgs::msg::Frame>(CAN_PUB_TOPIC, can_qos_pub);
    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      CAN_SUB_TOPIC,
      can_qos_sub,
      std::bind(&Stm32Node::can_callback, this, std::placeholders::_1),
      can_sub_options);

    led_cmd_sub_ = create_subscription<std_msgs::msg::UInt8>(
      led_cmd_topic,
      10,
      std::bind(&Stm32Node::led_callback, this, std::placeholders::_1));
    limit_sw_pub_ = create_publisher<std_msgs::msg::UInt8>(limit_sw_topic, 10);
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
      heartbeat_timed_out_ = false;
      return;
    }

    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    int16_t w = 0;
    if (protocol::decode_quaternion(*frame, x, y, z, w)) {
      publish_imu(x, y, z, w);
      return;
    }

    uint8_t limit_state = 0;
    if (protocol::decode_limit_switch(*frame, limit_state)) {
      std_msgs::msg::UInt8 output;
      output.data = limit_state;
      limit_sw_pub_->publish(output);
    }
  }

  /// @brief スケールを掛けたクォータニオンからIMUメッセージを送信する
  /// @param x X成分
  /// @param y Y成分
  /// @param z Z成分
  /// @param w W成分
  void publish_imu(int16_t x, int16_t y, int16_t z, int16_t w)
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

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = imu_frame_id_;
    imu.orientation.x = qx / norm;
    imu.orientation.y = qy / norm;
    imu.orientation.z = qz / norm;
    imu.orientation.w = qw / norm;
    imu.angular_velocity_covariance[0] = -1.0;//使用していないことを明記
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);
  }

  /// @brief LEDコマンドをSTM32へ送信する
  /// @param msg LEDコマンド
  void led_callback(const std_msgs::msg::UInt8::SharedPtr msg)
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
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limit_sw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr alive_timer_;

  std::string imu_frame_id_;
  double quaternion_scale_{1.0};
  std::chrono::steady_clock::time_point last_heartbeat_from_stm32_;
  std::chrono::milliseconds heartbeat_timeout_{0};
  bool heartbeat_timed_out_{false};
};

}  // namespace stm32_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stm32_driver::Stm32Node>());
  rclcpp::shutdown();
  return 0;
}
