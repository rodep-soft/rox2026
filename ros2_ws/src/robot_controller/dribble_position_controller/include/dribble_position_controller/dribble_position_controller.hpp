#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_controller/action/dribble_position.hpp"
#include "std_msgs/msg/float32.hpp"

class DribblePositionController : public rclcpp::Node
{
public:
  DribblePositionController();

private:
  using DribblePosition = robot_controller::action::DribblePosition;
  using GoalHandle = rclcpp_action::ServerGoalHandle<DribblePosition>;

  enum class State : uint8_t
  {
    DRIBBLE,
    INTAKE,
    SHOOT,
    HOLD_SHOOT,
    RETURN_TO_DRIBBLE,
  };

  void declare_parameters();
  void get_parameters();
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DribblePosition::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void position_feedback_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void timer_callback();
  void start_goal(const std::shared_ptr<GoalHandle> goal_handle);
  void set_target_position(double position_rad, State state);
  void finish_goal(bool succeeded, const std::string & message);

  double dribble_position_rad_{0.0};
  double intake_position_rad_{0.0};
  double shoot_position_rad_{0.0};
  double position_tolerance_rad_{0.02};
  double shoot_to_dribble_delay_sec_{1.0};
  double move_timeout_sec_{3.0};
  double feedback_timeout_sec_{0.5};
  int command_period_ms_{20};
  int qos_depth_{1};

  double target_position_rad_{0.0};
  double current_position_rad_{0.0};
  State state_{State::DRIBBLE};
  rclcpp::Time phase_start_time_;
  rclcpp::Time last_feedback_time_;
  std::string dribble_position_command_topic_;
  std::string dribble_position_feedback_topic_;
  std::string dribble_position_action_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_command_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_feedback_sub_;
  rclcpp_action::Server<DribblePosition>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
};
