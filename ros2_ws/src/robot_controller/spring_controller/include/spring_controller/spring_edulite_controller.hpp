#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/srv/set_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SpringEduliteController : public rclcpp::Node {
public:
  SpringEduliteController();

private:
  enum class State : uint8_t { HOMING, ZEROING, READY, ERROR };

  void declare_parameters();
  void get_parameters();

  void fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void limit_switch_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void actuator_state_callback(
      const actuator_msgs::msg::ActuatorState::SharedPtr msg);
  void timer_callback();

  void reset_for_homing();
  void request_zero_reference();
  void publish_target();

  State current_state_{State::HOMING};
  bool config_valid_{true};
  bool emergency_stop_active_{false};
  bool previous_fire_request_{false};
  bool is_limit_switch_on_{false};
  bool driver_ready_{false};
  bool position_reference_set_{false};
  bool zero_request_pending_{false};

  int limit_switch_bit_offset_{0};
  int command_period_ms_{10};
  int qos_depth_{1};
  int logical_id_{4};
  int zeroing_stable_feedback_count_{0};
  int zeroing_required_stable_feedback_count_{3};
  double fire_increment_rad_{-6.283185307};
  double homing_velocity_rad_s_{0.5};
  double homing_timeout_sec_{30.0};
  double zeroing_velocity_threshold_rad_s_{0.05};
  double target_position_rad_{0.0};
  std::string target_topic_{"/edulite/target"};
  std::string state_topic_{"/edulite/state"};
  std::string set_position_service_{"/edulite/set_position"};

  rclcpp::Time homing_start_time_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr limit_switch_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr
      actuator_state_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr
      spring_position_pub_;
  rclcpp::Client<actuator_msgs::srv::SetPosition>::SharedPtr
      set_position_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
