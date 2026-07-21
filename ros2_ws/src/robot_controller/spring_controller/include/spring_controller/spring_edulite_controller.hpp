#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

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
  bool previous_fire_request_{false};  // 立上り検出に使う、前回のspring_is_fire値。
  bool fire_pending_{false};  // READY状態で受理済みの発射要求。
  bool stop_dribble_on_fire_{true};  // 発射前にdribbleの停止を待つか。
  bool dribble_is_stopped_{false};  // dribble_controllerから受信した停止完了状態。
  double loading_velocity_rad_s_{0.0};  // LOAD状態で出力する目標角速度[rad/s]。
  double fire_velocity_rad_s_{0.0};  // FIRE状態で出力する目標角速度[rad/s]。
  double fire_duration_sec_{0.0};  // FIRE状態を継続する時間[s]。
  int qos_depth_{1};
  rclcpp::Time fire_start_time_;  // FIRE状態へ遷移した時刻。

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_request_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr limit_switch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_is_stopped_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spring_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_stop_request_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void limit_switch_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void dribble_is_stopped_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void start_fire();
  void publish_dribble_stop_request();
  void timer_callback();
};

#endif  // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
