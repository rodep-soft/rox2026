#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

#include "motor_can_bridge/msg/can_frame.hpp"

// motor controller nodesのPWM topicを受け取り、STMへ送るCAN frame形式に詰め替える。
// このnode自身はCAN busへ直接送らず、/can_tx にpublishしてCAN送信nodeへ渡す。
class MotorCanBridgeNode : public rclcpp::Node
{
public:
  MotorCanBridgeNode();

private:
  void DeclareParameters();
  void GetParameters();

  // 最新のPWM値を使って、一定周期でmabuchi用とmad motor用のCAN frameをpublishする。
  void TimerCallback();

  void MabuchiPwmCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void MadMotorPwmCallback(const std_msgs::msg::Int16::SharedPtr msg);

  // controllerから一定時間PWMが来ていない場合は、安全側として0を返す。
  int16_t GetPwmOrZeroOnTimeout(
    int16_t latest_pwm,
    const rclcpp::Time & last_update_time,
    const rclcpp::Time & now) const;

  // 8byte CAN payloadを作る。data[1..2]にint16 little-endianのPWMを入れる。
  motor_can_bridge::msg::CanFrame CreateMotorCanFrame(
    uint32_t can_id,
    int16_t pwm,
    const rclcpp::Time & stamp) const;
  
  // param
  std::string mabuchi_pwm_topic_;
  std::string mad_motor_pwm_topic_;
  std::string can_tx_topic_;
  uint32_t mabuchi_can_id_;
  uint32_t mad_motor_can_id_;
  int send_period_ms_;
  int timeout_ms_;

  // Subscribe callbackで更新され、TimerCallbackで周期送信に使われる最新値。
  int16_t latest_mabuchi_pwm_;
  int16_t latest_mad_motor_pwm_;
  rclcpp::Time last_mabuchi_time_;
  rclcpp::Time last_mad_motor_time_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mabuchi_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mad_motor_subscription_;
  rclcpp::Publisher<motor_can_bridge::msg::CanFrame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
