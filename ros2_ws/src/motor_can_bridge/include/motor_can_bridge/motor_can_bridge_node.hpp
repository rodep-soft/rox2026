#pragma once

#include <cstdint>
#include <string>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

// 1つのPWM topicを1つのCAN IDに変換するnode。
// 同じ実行ファイルをparameter違いで複数起動すると、複数モータに対応できる。
class MotorCanBridgeNode : public rclcpp::Node
{
public:
  MotorCanBridgeNode();

private:
  // DeclareParametersでparameter名と初期値を登録し、GetParametersで実際の値を読む。
  void DeclareParameters();
  void GetParameters();

  // subscriber、publisher、timerを作る。ROS 2との接続口をまとめて初期化する。
  void SetupRosInterfaces();

  // PWM topicにmessageが届いた時に呼ばれる処理。
  void PwmCallback(const std_msgs::msg::Int16::SharedPtr msg);

  // timer周期で呼ばれ、最新PWMからCAN frameをpublishする処理。
  void TimerCallback();

  // parameterで指定したmin/maxの範囲にPWM値を丸める。
  int16_t ClampPwm(int value) const;

  // PWM入力が一定時間途切れたら、安全側として0を返す。
  int16_t GetPwmOrZeroOnTimeout(const rclcpp::Time & now) const;

  // can_msgs/msg/Frameを作る。data[0..1]にint16 little-endianのPWMを入れる。
  can_msgs::msg::Frame CreateCanFrame(int16_t pwm, const rclcpp::Time & stamp) const;

  // config.yamlやlaunch fileから変更できるparameter値。
  std::string pwm_topic_;
  std::string can_tx_topic_;
  uint32_t can_id_;
  bool is_extended_;

  int min_pwm_;
  int max_pwm_;
  int send_period_ms_;
  int timeout_ms_;

  // PwmCallbackで更新され、TimerCallbackで周期送信に使われる最新値。
  // last_pwm_time_は、古いPWMを送り続けないためのtimeout判定に使う。
  int16_t latest_pwm_;
  rclcpp::Time last_pwm_time_;

  // PWM topicを受け取るsubscriber、CAN frameを出すpublisher、周期送信用timer。
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr pwm_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
