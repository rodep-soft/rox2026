#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

#include <cstdint>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/srv/set_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SpringEduliteController : public rclcpp::Node
{
public:
  SpringEduliteController();

private:
  enum class ControlState : uint8_t
  {
    HOMING,
    WAITING_FOR_STOP,
    WAITING_REARM,
    READY,
    ERROR,
  };

  void fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void limit_switch_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void actuator_state_callback(
    const actuator_msgs::msg::ActuatorState::SharedPtr msg);
  void control_timer_callback();

  void reset_for_homing();
  void request_zero_reference();
  void publish_target();

  ControlState control_state_{ControlState::HOMING};
  bool emergency_stop_active_{true};
  bool fire_request_active_{false};
  bool limit_switch_active_{false};
  bool actuator_ready_{false};
  bool position_reference_set_{false};
  bool zero_reference_request_pending_{false};

  int limit_switch_bit_offset_{0};
  int command_period_ms_{10};
  int stopped_feedback_count_{0};
  int required_stopped_feedback_count_{3};
  double fire_increment_rad_{-6.283185307};
  double homing_velocity_rad_s_{0.5};
  double homing_timeout_sec_{30.0};
  double zeroing_velocity_threshold_rad_s_{0.05};
  double target_position_rad_{0.0};
  uint16_t logical_id_{4};

  rclcpp::Time homing_started_at_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr limit_switch_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr
    actuator_state_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr
    position_command_pub_;
  rclcpp::Client<actuator_msgs::srv::SetPosition>::SharedPtr
    set_position_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
