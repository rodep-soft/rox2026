#ifndef BELT_DRIBBLE_CONTROLLER__BELT_DRIBBLE_CONTROLLER_HPP_
#define BELT_DRIBBLE_CONTROLLER__BELT_DRIBBLE_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

class BeltDribbleController : public rclcpp::Node {
 public:
  BeltDribbleController();

 private:
  enum class OperationMode : uint8_t {
    STOP = 0,
    DRIVE,
    INTAKE_AND_SHOOT,
    GAME2_MODE,
  };

  enum class BeltMode : uint8_t {
    STOP = 1,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
  };

  void declare_parameters();
  void get_parameters();
  void create_interfaces();
  void validate_parameters();

  void operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void game2_command_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void underbelt_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void upperbelt_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void dribble_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void timer_callback();

  int belt_target_rpm() const;
  int dribble_target_rpm() const;
  bool update_shoot_ready(int current_belt_target, int current_dribble_target,
                          const rclcpp::Time& current_time);
  bool is_rpm_valid(int rpm) const;
  void reset_shoot_ready();

  bool configuration_valid_{true};
  bool emergency_stop_active_{false};
  bool dribble_enabled_{false};
  bool underbelt_feedback_received_{false};
  bool upperbelt_feedback_received_{false};
  bool dribble_feedback_received_{false};
  bool shoot_ready_{false};
  OperationMode operation_mode_{OperationMode::STOP};
  BeltMode belt_mode_{BeltMode::STOP};
  int underbelt_current_rpm_{0};
  int upperbelt_current_rpm_{0};
  int dribble_current_rpm_{0};
  int stop_rpm_{0};
  int level_1_rpm_{3000};
  int level_2_rpm_{4000};
  int level_3_rpm_{5000};
  int dribble_on_rpm_{2000};
  int belt_rpm_tolerance_{100};
  int dribble_rpm_tolerance_{100};
  double ready_hold_sec_{0.1};
  int command_period_ms_{10};
  int qos_depth_{1};
  rclcpp::Time ready_since_{};

  std::string operation_mode_topic_;
  std::string belt_mode_topic_;
  std::string dribble_enabled_topic_;
  std::string game2_command_topic_;
  std::string emergency_stop_topic_;
  std::string underbelt_target_rpm_topic_;
  std::string upperbelt_target_rpm_topic_;
  std::string dribble_target_rpm_topic_;
  std::string underbelt_current_rpm_topic_;
  std::string upperbelt_current_rpm_topic_;
  std::string dribble_current_rpm_topic_;
  std::string intake_and_shoot_topic_;
  std::string shoot_ready_topic_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      operation_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      dribble_enabled_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      game2_command_subscription_;
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr intake_and_shoot_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_ready_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // BELT_DRIBBLE_CONTROLLER__BELT_DRIBBLE_CONTROLLER_HPP_
