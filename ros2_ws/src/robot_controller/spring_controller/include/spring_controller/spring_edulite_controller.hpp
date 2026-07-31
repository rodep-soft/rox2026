#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class SpringEduliteController : public rclcpp::Node
{
public:
  SpringEduliteController();

private:
  enum class State : uint8_t { READY, LOAD, FIRE, ERROR };
  enum class OperationMode : uint8_t { STOP, DRIVE, SHOT_CYCLE, BELT_ONLY };

  void declare_parameters();
  void get_parameters();
  void operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void limit_switch_callback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  bool spring_fire_allowed() const;
  void prepare_spring_for_stop();
  void start_loading();
  void start_fire();
  void timer_callback();
  const char * state_name(State state) const;
  const char * operation_mode_name(OperationMode mode) const;
  void log_fire_request_rejection() const;

  State now_state_{State::LOAD};
  OperationMode operation_mode_{OperationMode::STOP};
  int limit_switch_index_{0};
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
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr operation_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
    limit_switch_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spring_velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
