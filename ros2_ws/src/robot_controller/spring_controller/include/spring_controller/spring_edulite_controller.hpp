#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class SpringEduliteController : public rclcpp::Node
{
public:
  SpringEduliteController();

private:
  enum class State
  {
    READY,
    LOAD,
    FIRE,
  };

  State now_state_{State::LOAD};  // 現在のばね射出状態。
  int limit_switch_index_{0};  // 引き切り判定に使うリミットスイッチ配列のindex。
  bool is_configuration_valid_{true};  // パラメータが有効か。falseの場合は0 rad/sを出力する。
  bool is_loaded_{false};  // リミットスイッチで取得した、ばねを引き切った状態。
  bool emergency_stop_active_{false};  // 非常停止中は現在の安全状態で停止を維持する。
  bool previous_fire_request_{false};  // 立上り検出に使う、前回のspring_is_fire値。
  bool fire_pending_{false};  // READY状態で受理済みの発射要求。
  double loading_velocity_rad_s_{0.0};  // LOAD状態で出力する目標角速度[rad/s]。
  double fire_velocity_rad_s_{0.0};  // FIRE状態で出力する目標角速度[rad/s]。
  double fire_duration_sec_{0.0};  // FIRE状態を継続する時間[s]。
  int command_period_ms_{10};
  int qos_depth_{1};
  rclcpp::Time fire_start_time_;  // FIRE状態へ遷移した時刻。
  std::string fire_request_topic_;
  std::string emergency_stop_topic_;
  std::string limit_switch_topic_;
  std::string spring_velocity_command_topic_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr limit_switch_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spring_velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void declare_parameters();
  void get_parameters();
  void fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void limit_switch_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void start_fire();
  void timer_callback();
};

#endif  // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
