#ifndef BELT_DRIBBLE_CONTROLLER__BELT_DRIBBLE_CONTROLLER_HPP_
#define BELT_DRIBBLE_CONTROLLER__BELT_DRIBBLE_CONTROLLER_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

class BeltDribbleController : public rclcpp::Node
{
 public:
  BeltDribbleController();

 private:
  static constexpr int stop_rpm = 0;

  enum class OperationMode : uint8_t
  {
    STOP,
    DRIVE,
    SHOT_CYCLE,
    BELT_ONLY,
  };

  enum class BeltMode : uint8_t
  {
    STOP,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4,
    LEVEL_5,
    LEVEL_6,
  };

  void declare_parameters();
  void get_parameters();
  void create_interfaces();
  void validate_parameters();

  // /operation_mode受信時に呼ばれる。不正値はSTOPとして扱い、モードが変わった場合は
  // 回転到達の保持時間を破棄してshot cycle開始可能状態を解除する。
  void operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  // /belt/mode受信時に呼ばれる。不正値はSTOPにし、変更時はshot cycle開始可能状態を解除する。
  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  // /dribble/enabled受信時に呼ばれる。ON/OFF変更時はshot cycle開始可能状態を解除する。
  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  // /shot_cycle/requestのtrue受信時に呼ばれる。非常停止中、SHOT_CYCLE以外、または
  // 3モータのRPMが目標へready_hold_sec以上到達していない場合は拒否する。許可時だけ/startへtrueをpublishする。
  void shot_cycle_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  // /emergency_stop受信時に呼ばれる。trueならshot cycle開始可能状態を解除し、timerから停止RPMを出す。
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void underbelt_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void upperbelt_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void dribble_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  // 設定周期で呼ばれる。設定不正・非常停止・STOPでは全目標を0 RPMにし、それ以外では
  // 選択モードの目標RPMを3つの/target/rpmへpublishする。その後、freshなfeedbackで開始可否を更新する。
  void timer_callback();

  // belt_mode_に対応するRPMを返す。不明値は安全側として0 RPMを返す。
  int belt_target_rpm() const;
  // BELT_ONLYまたはdribble_enabled_がfalseなら0 RPM、それ以外はdribble_on_rpm_を返す。
  int dribble_target_rpm() const;
  // SHOT_CYCLE中、非常停止でなく、3つのfeedbackがfreshで、全RPMが許容誤差内の状態を
  // ready_hold_sec継続したときだけtrueを返す。途中で条件が崩れたら保持時間をリセットする。
  bool update_shoot_ready(int current_belt_target, int current_dribble_target,
                          const rclcpp::Time& current_time);
  void log_shot_rejection(int current_belt_target, int current_dribble_target,
                          const rclcpp::Time& current_time) const;
  // feedbackを一度でも受信し、受信からfeedback_timeout_sec以内ならtrueを返す。
  bool feedback_is_fresh(bool received, const rclcpp::Time& received_at,
                         const rclcpp::Time& current_time) const;
  void update_feedback_timeout_state(const rclcpp::Time& current_time);
  const char* operation_mode_name(OperationMode mode) const;
  const char* belt_mode_name(BeltMode mode) const;
  bool is_rpm_valid(int rpm) const;
  // shot cycleの開始許可とRPM到達保持開始時刻を消去する。
  void reset_shoot_ready();

  bool configuration_valid_{true};
  bool emergency_stop_active_{false};
  bool dribble_enabled_{false};
  bool underbelt_feedback_received_{false};
  bool upperbelt_feedback_received_{false};
  bool dribble_feedback_received_{false};
  bool underbelt_feedback_timed_out_{false};
  bool upperbelt_feedback_timed_out_{false};
  bool dribble_feedback_timed_out_{false};
  bool shoot_ready_{false};
  bool command_log_initialized_{false};
  OperationMode operation_mode_{OperationMode::STOP};
  BeltMode belt_mode_{BeltMode::STOP};
  OperationMode last_logged_operation_mode_{OperationMode::STOP};
  BeltMode last_logged_belt_mode_{BeltMode::STOP};
  bool last_logged_dribble_enabled_{false};
  bool last_logged_emergency_stop_active_{false};
  int underbelt_current_rpm_{0};
  int upperbelt_current_rpm_{0};
  int dribble_current_rpm_{0};
  int last_logged_belt_target_rpm_{0};
  int last_logged_dribble_target_rpm_{0};
  int level_1_rpm_{3000};
  int level_2_rpm_{3500};
  int level_3_rpm_{4000};
  int level_4_rpm_{4500};
  int level_5_rpm_{5000};
  int level_6_rpm_{5500};
  int dribble_on_rpm_{2000};
  int belt_rpm_tolerance_{100};
  int dribble_rpm_tolerance_{100};
  double ready_hold_sec_{0.1};
  double feedback_timeout_sec_{0.5};
  int command_period_ms_{10};
  int qos_depth_{1};
  rclcpp::Time ready_since_{};
  rclcpp::Time underbelt_feedback_received_at_{};
  rclcpp::Time upperbelt_feedback_received_at_{};
  rclcpp::Time dribble_feedback_received_at_{};

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      operation_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      dribble_enabled_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      shot_cycle_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      emergency_stop_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr
      underbelt_feedback_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr
      upperbelt_feedback_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr
      dribble_feedback_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr
      underbelt_target_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr
      upperbelt_target_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr dribble_target_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shot_cycle_start_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // BELT_DRIBBLE_CONTROLLER__BELT_DRIBBLE_CONTROLLER_HPP_
