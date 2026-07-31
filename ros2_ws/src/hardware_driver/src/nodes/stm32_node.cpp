/* ROS 2とSTM32間でCANフレームを送受信するノード。 */
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "stm32_driver/stm32_protocol.hpp"

namespace stm32_driver
{

using namespace std::chrono_literals;

constexpr char CAN_PUB_TOPIC[] = "/socketcan_bridge/tx";
constexpr char CAN_SUB_TOPIC[] = "/socketcan_bridge/rx";

class Stm32Node : public rclcpp::Node
{
public:
  Stm32Node()
  : Node("stm32_driver_node"),
    last_heartbeat_from_stm32_(std::chrono::steady_clock::now()),
    is_heartbeat_timed_out_(false)
  {
    const auto motor_target_rpm_topics =
      declare_parameter<std::vector<std::string>>(
      "motor_target_rpm_topics",
      {"/stm32/underbelt/target/rpm", "/stm32/upperbelt/target/rpm",
        "/stm32/dribble/target/rpm"});
    const auto motor_current_rpm_topics =
      declare_parameter<std::vector<std::string>>(
      "motor_current_rpm_topics",
      {"/stm32/underbelt/current/rpm", "/stm32/upperbelt/current/rpm",
        "/stm32/dribble/current/rpm"});
    const auto led_cmd_topic =
      declare_parameter<std::string>("led_cmd_topic", "/led/cmd");
    const auto limit_sw_topic =
      declare_parameter<std::string>("limit_sw_topic", "/limit_switches");
    const auto keep_alive_period_ms =
      declare_parameter<int64_t>("keep_alive_period_ms", 100);

    const auto timeout_ms = declare_parameter<int64_t>("timeout_ms", 500);

    if (keep_alive_period_ms <= 0 || timeout_ms <= 0) {
      throw std::invalid_argument("timer parameters must be greater than zero");
    }

    if (motor_target_rpm_topics.size() != protocol::MOTOR_NUM ||
      motor_current_rpm_topics.size() != protocol::MOTOR_NUM)
    {
      throw std::invalid_argument(
              "motor RPM topic arrays must contain one entry for each STM32 motor");
    }

    heartbeat_timeout_ = std::chrono::milliseconds(timeout_ms);

    auto can_qos_pub =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto can_qos =
      rclcpp::QoS(rclcpp::KeepLast(30)).best_effort().durability_volatile();
    can_pub_ =
      create_publisher<can_msgs::msg::Frame>(CAN_PUB_TOPIC, can_qos_pub);

    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      CAN_SUB_TOPIC, can_qos,
      std::bind(&Stm32Node::can_callback, this, std::placeholders::_1));

    for (std::size_t motor = 0; motor < protocol::MOTOR_NUM; ++motor) {
      motor_target_rpm_subs_[motor] = create_subscription<std_msgs::msg::Int16>(
        motor_target_rpm_topics[motor], 10,
        [this, motor](const std_msgs::msg::Int16::SharedPtr msg) {
          motor_target_callback(motor, msg);
        });
      motor_current_rpm_pubs_[motor] = create_publisher<std_msgs::msg::Int16>(
        motor_current_rpm_topics[motor], 10);
    }

    led_cmd_sub_ = create_subscription<std_msgs::msg::UInt8>(
      led_cmd_topic, 10,
      std::bind(&Stm32Node::led_callback, this, std::placeholders::_1));

    limit_sw_pub_ =
      create_publisher<std_msgs::msg::UInt8MultiArray>(limit_sw_topic, 10);

    alive_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(keep_alive_period_ms),
      std::bind(&Stm32Node::alive_timer_callback, this));

    RCLCPP_INFO(get_logger(), "stm32_driver_node started");
  }

private:
  /// @brief canよりデータが流れたときにIDに応じてデコードを行う
  /// @param frame 受け取ったcanFrame
  void can_callback(const can_msgs::msg::Frame::SharedPtr frame)
  {
    if (!protocol::is_standard_data_frame(*frame)) {
      return;
    }

    const auto id = frame->id;
    if (id != protocol::HEARTBEAT_FROM_STM &&
      id != protocol::LIMIT_SWITCH_STATE &&
      (id < protocol::MOTOR_CURRENT_RPM_BASE ||
      id >= protocol::MOTOR_CURRENT_RPM_BASE + protocol::MOTOR_NUM))
    {
      return;
    }

    if (protocol::is_heartbeat_response(*frame)) {
      last_heartbeat_from_stm32_ = std::chrono::steady_clock::now();
      if (is_heartbeat_timed_out_) {
        RCLCPP_INFO(get_logger(), "Resuming after timeout");
        is_heartbeat_timed_out_ = false;
      }
      return;
    }

    std::size_t motor = 0;
    int16_t rpm = 0;
    if (protocol::decode_motor_current(*frame, motor, rpm)) {
      std_msgs::msg::Int16 output;
      output.data = rpm;
      motor_current_rpm_pubs_[motor]->publish(output);
      return;
    }

    uint8_t limit_state = 0;
    if (protocol::decode_limit_switch(*frame, limit_state)) {
      // STM32から受け取った1バイトを、bitごとに1スイッチとして配列へ展開する。
      // data[i]がi番目のリミットスイッチに対応し、購読側はindexでスイッチを選ぶ。
      std_msgs::msg::UInt8MultiArray output;
      output.data.resize(8);
      for (std::size_t bit = 0; bit < output.data.size(); ++bit) {
        output.data[bit] = static_cast<uint8_t>((limit_state >> bit) & 0x01);
      }
      limit_sw_pub_->publish(output);
    }
  }

  /// @brief 制御ノードから受け取った目標RPMを対応するSTM32モーターへ送信する
  /// @param motor モーターのインデックス
  /// @param msg 目標RPM
  void motor_target_callback(
    std::size_t motor,
    const std_msgs::msg::Int16::SharedPtr msg)
  {
    can_pub_->publish(
      protocol::make_motor_target_frame(
        motor, static_cast<int16_t>(msg->data)));
  }

  /// @brief LEDのコマンドをstm32へ送信
  /// @param msg
  void led_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    can_pub_->publish(protocol::make_led_frame(msg->data));
  }

  /// @brief stm32へ空のフレームを送り，生存報告とstm32が生きているのかの判断
  void alive_timer_callback()
  {
    can_pub_->publish(protocol::make_alive_frame());

    const auto elapsed =
      std::chrono::steady_clock::now() - last_heartbeat_from_stm32_;
    if (elapsed > heartbeat_timeout_) {
      if (!is_heartbeat_timed_out_) {
        RCLCPP_WARN(get_logger(), "STM32 heartbeat timed out");
        is_heartbeat_timed_out_ = true;
      }
    }
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  std::array<rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr,
    protocol::MOTOR_NUM>
  motor_target_rpm_subs_;
  std::array<rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr,
    protocol::MOTOR_NUM>
  motor_current_rpm_pubs_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr limit_sw_pub_;
  rclcpp::TimerBase::SharedPtr alive_timer_;

  std::chrono::steady_clock::time_point last_heartbeat_from_stm32_;
  std::chrono::milliseconds heartbeat_timeout_;
  bool is_heartbeat_timed_out_;
};

}  // namespace stm32_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stm32_driver::Stm32Node>());
  rclcpp::shutdown();
  return 0;
}
