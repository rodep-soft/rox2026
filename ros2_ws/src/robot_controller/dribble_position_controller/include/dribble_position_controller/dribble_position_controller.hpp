#pragma once

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class DribblePositionController : public rclcpp::Node
{
public:
  DribblePositionController();

private:
  enum class Position : uint8_t
  {
    DRIBBLE,
    INTAKE,
    SHOOT,
    OPEN,
  };

  enum class State : uint8_t
  {
    IDLE,
    MANUAL_MOVE,
    INTAKE,
    SHOOT,
    HOLD_SHOOT,
    RETURN_TO_DRIBBLE
  };

  void declare_parameters();
  void get_parameters();
  void position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void shot_cycle_start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void position_feedback_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void watchdog_callback();
  void set_target_position(double position_rad, State state);

  double dribble_position_rad_{0.0};
  double intake_position_rad_{0.0};
  double shoot_position_rad_{0.0};
  double open_position_rad_{0.0};
  double position_tolerance_rad_{0.02};
  double shoot_to_dribble_delay_sec_{1.0};
  double move_timeout_sec_{3.0};
  double feedback_timeout_sec_{0.5};
  int command_period_ms_{20};
  int qos_depth_{1};
  double target_position_rad_{0.0};
  State state_{State::IDLE};
  bool emergency_stop_active_{false};
  rclcpp::Time phase_start_time_;
  rclcpp::Time last_feedback_time_;
  std::string dribble_position_command_topic_;
  std::string dribble_position_feedback_topic_;
  std::string dribble_position_mode_topic_;
  std::string shot_cycle_start_topic_;
  std::string emergency_stop_topic_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_command_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr position_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shot_cycle_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    position_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};
