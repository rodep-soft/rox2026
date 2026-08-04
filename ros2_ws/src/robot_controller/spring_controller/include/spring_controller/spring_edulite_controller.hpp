#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SpringEduliteController : public rclcpp::Node
{
public:
  SpringEduliteController();

private:
  enum class State : uint8_t
  {
    READY,
    LOAD,
    FIRE,
    ERROR
  };

  void declare_parameters();
  void get_parameters();

  // /spring/fire_requestの立上りで呼ばれる。設定正常・非常停止なし・READY・装填完了の
  // 全条件を満たす場合だけ発射待ちにする。それ以外は理由をログに出して無視する。
  void fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  // /emergency_stop受信時に呼ばれる。trueならFIREを中断し、次のtimerでLOAD/READYに対応する速度を出す。
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void limit_switch_callback(const std_msgs::msg::UInt8::SharedPtr msg);

  // 設定周期で呼ばれる。設定不正なら0 rad/sを出し、正常時はLOAD/READY/FIRE/ERRORを遷移して
  // /spring/vel_commandへ速度をpublishする。LOAD timeout時はERRORへ移行する。
  void timer_callback();

  // 設定正常・非常停止なしのときだけtrueを返す。
  bool spring_fire_allowed() const;
  // 発射待ちを解除し、装填済みならREADY、未装填ならLOADへ戻す。ERRORは維持する。
  void prepare_spring_for_stop();
  void start_loading();
  void start_fire();
  // ログ表示や拒否理由判定のための小さな補助関数。
  const char * state_name(State state) const;
  void log_fire_request_rejection() const;

  State now_state_{State::LOAD};
  int limit_switch_bit_offset_{0};
  bool is_configuration_valid_{true};
  bool is_loaded_{false};
  bool emergency_stop_active_{false};
  bool previous_fire_request_{false};
  bool fire_pending_{false};
  bool limit_switch_received_{false};
  uint8_t last_limit_switch_value_{0};
  double loading_velocity_rad_s_{0.0};
  double fire_velocity_rad_s_{0.0};
  double fire_duration_sec_{0.0};
  double load_timeout_sec_{5.0};
  int command_period_ms_{10};
  int qos_depth_{1};
  rclcpp::Time fire_start_time_;
  rclcpp::Time load_start_time_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr limit_switch_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spring_velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
