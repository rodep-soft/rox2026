#ifndef LED_CONTROLLER__LED_CONTROLLER_HPP_
#define LED_CONTROLLER__LED_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>

#include "actuator_msgs/msg/actuator_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "robot_msgs/msg/game2_state.hpp"
#include "robot_msgs/msg/shot_cycle_state.hpp"
#include "robot_msgs/msg/spring_operation_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"

class LedControllerNode : public rclcpp::Node
{
public:
  LedControllerNode();

private:
  enum class DisplayMode : uint8_t
  {
    STARTUP = 0,
    READY = 1,
    EMERGENCY_STOP = 2,
    SHOT_OPENING = 3,
    LOADING = 4,
    FIRING = 5,
    RETURNING = 6,
    GAME2_SEARCHING = 7,
    GAME2_ALIGNING = 8,
    ERROR = 9,
    ARM_DRIBBLE = 10,
    SLOW_FIRING = 11,
    ARM_FEED = 12,
    ARM_RECEIVE = 13,
    ARM_HOME = 14,
    BELT_SPINUP = 15,
    BELT_OFFSET_MINUS_3 = 16,
    BELT_OFFSET_MINUS_2 = 17,
    BELT_OFFSET_MINUS_1 = 18,
    BELT_OFFSET_ZERO = 19,
    BELT_OFFSET_PLUS_1 = 20,
    BELT_OFFSET_PLUS_2 = 21,
    BELT_OFFSET_PLUS_3 = 22,
  };

  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void belt_mode_callback(const robot_msgs::msg::BeltMode::SharedPtr msg);
  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void drive_reversed_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void arm_position_callback(const robot_msgs::msg::ArmPosition::SharedPtr msg);
  void roller_target_callback(const actuator_msgs::msg::ActuatorTarget::SharedPtr msg);
  void spring_operation_state_callback(
    const robot_msgs::msg::SpringOperationState::SharedPtr msg);
  void shot_cycle_state_callback(const robot_msgs::msg::ShotCycleState::SharedPtr msg);
  void game2_state_callback(const robot_msgs::msg::Game2State::SharedPtr msg);
  void spring_fire_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_timer_callback();

  DisplayMode select_display_mode() const;
  uint8_t make_status_flags() const;
  DisplayMode belt_offset_display_mode() const;

  bool emergency_stop_received_{false};
  bool emergency_stop_active_{true};
  bool dribble_enabled_{false};
  bool drive_reversed_{false};
  bool spring_fire_request_active_{false};
  uint8_t belt_mode_{0};
  int8_t belt_rpm_offset_steps_{0};
  uint8_t arm_position_{robot_msgs::msg::ArmPosition::DRIBBLE};
  uint8_t spring_operation_state_{robot_msgs::msg::SpringOperationState::IDLE};
  uint8_t shot_cycle_state_{0};
  uint16_t roller_logical_id_{12};
  float roller_target_rpm_{0.0F};
  uint8_t game2_state_{0};
  std::chrono::steady_clock::time_point firing_display_until_{};
  std::chrono::steady_clock::time_point belt_offset_display_until_{};
  std::chrono::milliseconds firing_display_duration_{500};
  std::chrono::milliseconds belt_offset_display_duration_{1000};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_reversed_sub_;
  rclcpp::Subscription<robot_msgs::msg::ArmPosition>::SharedPtr arm_position_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTarget>::SharedPtr roller_target_sub_;
  rclcpp::Subscription<robot_msgs::msg::SpringOperationState>::SharedPtr
    spring_operation_state_sub_;
  rclcpp::Subscription<robot_msgs::msg::ShotCycleState>::SharedPtr shot_cycle_state_sub_;
  rclcpp::Subscription<robot_msgs::msg::Game2State>::SharedPtr game2_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spring_fire_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr led_command_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

#endif  // LED_CONTROLLER__LED_CONTROLLER_HPP_
