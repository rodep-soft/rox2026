/* ROS 2 <-> STM32 CAN frame encoder/decoder node. */
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "stm32_driver/stm32_protocol.hpp"

namespace stm32_driver
{

using namespace std::chrono_literals;

class Stm32Node : public rclcpp::Node
{
public:
  Stm32Node()
  : Node("stm32_driver_node"), current_rpm_{}, last_heartbeat_rx_(now())
  {
    const auto can_pub_topic = declare_parameter<std::string>("can_pub_topic", "/CAN/can0/transmit");
    const auto can_sub_topic = declare_parameter<std::string>("can_sub_topic", "/CAN/can0/receive");
    const auto target_rpm_topic = declare_parameter<std::string>("target_rpm_topic", "/target_rpm");
    const auto current_rpm_topic = declare_parameter<std::string>("current_rpm_topic", "/current_rpm");
    const auto led_cmd_topic = declare_parameter<std::string>("led_cmd_topic", "/led_cmd");
    const auto limit_sw_topic = declare_parameter<std::string>("limit_sw_topic", "/limit_sw");
    const auto keep_alive_period_ms = declare_parameter<int64_t>("keep_alive_period_ms", 100);
    const auto publish_period_ms = declare_parameter<int64_t>("publish_period_ms", 10);
    
    timeout_ms_ = declare_parameter<int64_t>("timeout_ms", 500);

    if (keep_alive_period_ms <= 0 || publish_period_ms <= 0 || timeout_ms_ <= 0) {
      throw std::invalid_argument("timer parameters must be greater than zero");
    }

    can_pub_ = create_publisher<can_msgs::msg::Frame>(can_pub_topic, 10);

    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      can_sub_topic, 10,
      std::bind(&Stm32Node::can_callback, this, std::placeholders::_1));

    target_rpm_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      target_rpm_topic, 10,
      std::bind(&Stm32Node::motor_target_callback, this, std::placeholders::_1));

    led_cmd_sub_ = create_subscription<std_msgs::msg::UInt8>(
      led_cmd_topic, 10,
      std::bind(&Stm32Node::led_callback, this, std::placeholders::_1));

    current_rpm_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      current_rpm_topic, 10);

    limit_sw_pub_ = create_publisher<std_msgs::msg::UInt8>(limit_sw_topic, 10);

    alive_timer_ = create_wall_timer(
      std::chrono::milliseconds(keep_alive_period_ms),
      std::bind(&Stm32Node::alive_timer_callback, this));
      
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&Stm32Node::publish_timer_callback, this));

    RCLCPP_INFO(get_logger(), "stm32_driver_node started");
  }

private:
  /// @brief canよりデータが流れたときにIDに応じてデコードを行う
  /// @param frame 受け取ったcanFrame
  void can_callback(const can_msgs::msg::Frame::SharedPtr frame)
  {
    if (protocol::is_heartbeat_response(*frame)) {
      last_heartbeat_rx_ = now();
      heartbeat_timed_out_ = false;
      return;
    }

    std::size_t motor = 0;
    float rpm = 0.0F;
    if (protocol::decode_motor_current(*frame, motor, rpm)) {
      current_rpm_[motor] = rpm;
      return;
    }

    uint8_t limit_state = 0;
    if (protocol::decode_limit_switch(*frame, limit_state)) {
      std_msgs::msg::UInt8 output;
      output.data = limit_state;
      limit_sw_pub_->publish(output);
    }
  }

  /// @brief 制御nodeから送られてきた目標rpmをstm32に送信
  /// @param msg モータID = MOTOR_TARGET_BASE + モータインデックス
  void motor_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != protocol::MOTOR_NUM) {
      RCLCPP_WARN(
        get_logger(), "target RPM requires %zu values; received %zu",
        protocol::MOTOR_NUM, msg->data.size());
    }

    const auto count = std::min(msg->data.size(), protocol::MOTOR_NUM);
    for (std::size_t motor = 0; motor < count; ++motor) {
      can_pub_->publish(protocol::make_motor_target_frame(motor, msg->data[motor]));
    }
  }

  /// @brief LEDのコマンドをstm32へ送信
  /// @param msg 
  void led_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    can_pub_->publish(protocol::make_led_frame(msg->data));
  }

  /// @brief stm32へ空のフレームを送り，生存報告
  void alive_timer_callback()
  {
    can_pub_->publish(protocol::make_alive_frame());

    if ((now() - last_heartbeat_rx_).nanoseconds() >
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::milliseconds(timeout_ms_)).count())
    {
      if (!heartbeat_timed_out_) {
        RCLCPP_WARN(get_logger(), "STM32 heartbeat timed out");
        heartbeat_timed_out_ = true;
      }
    }
  }

  /// @brief 受け取っている現在のrpmを一定周期でtopicに送信する
  void publish_timer_callback()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(current_rpm_.begin(), current_rpm_.end());
    current_rpm_pub_->publish(msg);
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_rpm_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_rpm_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr led_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limit_sw_pub_;
  rclcpp::TimerBase::SharedPtr alive_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::array<float, protocol::MOTOR_NUM> current_rpm_;
  rclcpp::Time last_heartbeat_rx_;
  int64_t timeout_ms_;
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
