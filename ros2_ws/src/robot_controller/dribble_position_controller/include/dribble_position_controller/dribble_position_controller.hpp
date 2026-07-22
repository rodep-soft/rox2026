#pragma once

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

  enum class State : uint8_t {DRIBBLE, INTAKE, SHOOT};

  void declare_parameters();
  void get_parameters();
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DribblePosition::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void position_feedback_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void publish_target_position(double position_rad);
  void finish_goal(bool success, const std::string & message);

  double dribble_position_rad_{0.0};
  double intake_position_rad_{0.0};
  double shoot_position_rad_{0.0};
  double position_tolerance_rad_{0.02};
  double current_position_rad_{0.0};
  double target_position_rad_{0.0};
  int qos_depth_{1};
  State state_{State::DRIBBLE};
  std::string dribble_position_command_topic_;
  std::string dribble_position_feedback_topic_;
  std::string dribble_position_action_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_command_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_feedback_sub_;
  rclcpp_action::Server<DribblePosition>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
};
