#include "dribble_position_controller/dribble_position_controller.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <utility>

DribblePositionController::DribblePositionController()
: Node("dribble_position_controller")
{
  const auto command_topic = declare_parameter<std::string>(
    "dribble_position_command_topic", "/dribble/position_command");
  const auto feedback_topic = declare_parameter<std::string>(
    "dribble_position_feedback_topic", "/dribble/position_feedback");
  const auto action_name = declare_parameter<std::string>(
    "dribble_position_action", "/dribble/position");
  dribble_position_rad_ = declare_parameter<double>("dribble_position_rad", 0.0);
  intake_position_rad_ = declare_parameter<double>("intake_position_rad", 1.5);
  shoot_position_rad_ = declare_parameter<double>("shoot_position_rad", 2.0);
  intake_offset_rad_ = declare_parameter<double>("intake_offset_rad", 1.0);
  position_tolerance_rad_ = declare_parameter<double>("position_tolerance_rad", 0.02);

  position_command_pub_ = create_publisher<std_msgs::msg::Float32>(command_topic, 10);
  position_feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
    feedback_topic, 10,
    std::bind(&DribblePositionController::position_feedback_callback, this, std::placeholders::_1));
  action_server_ = rclcpp_action::create_server<DribblePosition>(
    this, action_name,
    std::bind(
      &DribblePositionController::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&DribblePositionController::handle_cancel, this, std::placeholders::_1),
    std::bind(&DribblePositionController::handle_accepted, this, std::placeholders::_1));

  publish_target_position(dribble_position_rad_);
}

rclcpp_action::GoalResponse DribblePositionController::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const DribblePosition::Goal> goal)
{
  if (goal->command != DribblePosition::Goal::DRIBBLE &&
    goal->command != DribblePosition::Goal::SHOOT)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DribblePositionController::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  if (goal_handle == active_goal_) {
    finish_goal(false, "Goal cancelled");
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DribblePositionController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  if (active_goal_) {
    finish_goal(false, "Preempted by a new goal");
  }
  active_goal_ = goal_handle;
  if (goal_handle->get_goal()->command == DribblePosition::Goal::DRIBBLE) {
    state_ = State::DRIBBLE;
    publish_target_position(dribble_position_rad_);
  } else {
    state_ = State::OFFSET;
    publish_target_position(dribble_position_rad_ + intake_offset_rad_);
  }
}

void DribblePositionController::position_feedback_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  current_position_rad_ = msg->data;
  if (!active_goal_) {
    return;
  }

  auto feedback = std::make_shared<DribblePosition::Feedback>();
  feedback->state = static_cast<uint8_t>(state_);
  feedback->target_position_rad = static_cast<float>(target_position_rad_);
  feedback->current_position_rad = static_cast<float>(current_position_rad_);
  active_goal_->publish_feedback(feedback);

  if (std::abs(current_position_rad_ - target_position_rad_) > position_tolerance_rad_) {
    return;
  }

  switch (state_) {
    case State::DRIBBLE:
      finish_goal(true, "Reached dribble position");
      break;
    case State::OFFSET:
      state_ = State::INTAKE;
      publish_target_position(intake_position_rad_);
      break;
    case State::INTAKE:
      state_ = State::SHOOT;
      publish_target_position(shoot_position_rad_);
      break;
    case State::SHOOT:
      break;
  }
}

void DribblePositionController::publish_target_position(double position_rad)
{
  target_position_rad_ = position_rad;
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(position_rad);
  position_command_pub_->publish(command);
}

void DribblePositionController::finish_goal(bool success, const std::string & message)
{
  if (!active_goal_) {
    return;
  }
  auto result = std::make_shared<DribblePosition::Result>();
  result->success = success;
  result->message = message;
  if (success) {
    active_goal_->succeed(result);
  } else {
    active_goal_->abort(result);
  }
  active_goal_.reset();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribblePositionController>());
  rclcpp::shutdown();
  return 0;
}
