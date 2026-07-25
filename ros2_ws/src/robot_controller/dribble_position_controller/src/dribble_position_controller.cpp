#include "dribble_position_controller/dribble_position_controller.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

DribblePositionController::DribblePositionController()
: Node("dribble_position_controller")
{
  declare_parameters();
  get_parameters();

  if (command_period_ms_ <= 0) {
    RCLCPP_WARN(get_logger(), "command_period_ms must be positive. Using 20 ms.");
    command_period_ms_ = 20;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using 1.");
    qos_depth_ = 1;
  }
  if (!std::isfinite(position_tolerance_rad_) || position_tolerance_rad_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "position_tolerance_rad must be positive. Using 0.02 rad.");
    position_tolerance_rad_ = 0.02;
  }
  if (!std::isfinite(shoot_to_dribble_delay_sec_) || shoot_to_dribble_delay_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "shoot_to_dribble_delay_sec must be non-negative. Using 1.0 s.");
    shoot_to_dribble_delay_sec_ = 1.0;
  }
  if (!std::isfinite(move_timeout_sec_) || move_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "move_timeout_sec must be positive. Using 3.0 s.");
    move_timeout_sec_ = 3.0;
  }
  if (!std::isfinite(feedback_timeout_sec_) || feedback_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "feedback_timeout_sec must be positive. Using 0.5 s.");
    feedback_timeout_sec_ = 0.5;
  }

  target_position_rad_ = dribble_position_rad_;
  position_command_pub_ = create_publisher<std_msgs::msg::Float32>(
    dribble_position_command_topic_, rclcpp::QoS(qos_depth_));
  position_feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
    dribble_position_feedback_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&DribblePositionController::position_feedback_callback, this, std::placeholders::_1));
  action_server_ = rclcpp_action::create_server<DribblePosition>(
    this, dribble_position_action_,
    std::bind(&DribblePositionController::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&DribblePositionController::handle_cancel, this, std::placeholders::_1),
    std::bind(&DribblePositionController::handle_accepted, this, std::placeholders::_1));
  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribblePositionController::timer_callback, this));
}

void DribblePositionController::declare_parameters()
{
  declare_parameter<std::string>("dribble_position_command_topic", "/dribble/position_command");
  declare_parameter<std::string>("dribble_position_feedback_topic", "/dribble/position_feedback");
  declare_parameter<std::string>("dribble_position_action", "/dribble/position");
  declare_parameter<double>("dribble_position_rad", 0.0);
  declare_parameter<double>("intake_position_rad", 1.5);
  declare_parameter<double>("shoot_position_rad", 2.0);
  declare_parameter<double>("position_tolerance_rad", 0.02);
  declare_parameter<double>("shoot_to_dribble_delay_sec", 1.0);
  declare_parameter<double>("move_timeout_sec", 3.0);
  declare_parameter<double>("feedback_timeout_sec", 0.5);
  declare_parameter<int>("command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
}

void DribblePositionController::get_parameters()
{
  get_parameter("dribble_position_command_topic", dribble_position_command_topic_);
  get_parameter("dribble_position_feedback_topic", dribble_position_feedback_topic_);
  get_parameter("dribble_position_action", dribble_position_action_);
  get_parameter("dribble_position_rad", dribble_position_rad_);
  get_parameter("intake_position_rad", intake_position_rad_);
  get_parameter("shoot_position_rad", shoot_position_rad_);
  get_parameter("position_tolerance_rad", position_tolerance_rad_);
  get_parameter("shoot_to_dribble_delay_sec", shoot_to_dribble_delay_sec_);
  get_parameter("move_timeout_sec", move_timeout_sec_);
  get_parameter("feedback_timeout_sec", feedback_timeout_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
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
    target_position_rad_ = dribble_position_rad_;
    auto result = std::make_shared<DribblePosition::Result>();
    result->success = false;
    result->message = "Goal cancelled; returning to dribble position";
    active_goal_->canceled(result);
    active_goal_.reset();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DribblePositionController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  if (active_goal_) {
    finish_goal(false, "Preempted by a new goal");
  }
  start_goal(goal_handle);
}

void DribblePositionController::start_goal(const std::shared_ptr<GoalHandle> goal_handle)
{
  active_goal_ = goal_handle;
  last_feedback_time_ = now();
  if (goal_handle->get_goal()->command == DribblePosition::Goal::DRIBBLE) {
    set_target_position(dribble_position_rad_, State::DRIBBLE);
  } else {
    set_target_position(intake_position_rad_, State::INTAKE);
  }
}

void DribblePositionController::position_feedback_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring non-finite position feedback");
    return;
  }

  current_position_rad_ = msg->data;
  last_feedback_time_ = now();
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
    case State::INTAKE:
      set_target_position(shoot_position_rad_, State::SHOOT);
      break;
    case State::SHOOT:
      state_ = State::HOLD_SHOOT;
      phase_start_time_ = now();
      break;
    case State::HOLD_SHOOT:
    case State::RETURN_TO_DRIBBLE:
      if (state_ == State::RETURN_TO_DRIBBLE) {
        finish_goal(true, "Reached dribble position after shoot");
      }
      break;
  }
}

void DribblePositionController::timer_callback()
{
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(target_position_rad_);
  position_command_pub_->publish(command);

  if (!active_goal_) {
    return;
  }

  const auto current_time = now();
  if ((current_time - last_feedback_time_).seconds() > feedback_timeout_sec_) {
    target_position_rad_ = dribble_position_rad_;
    finish_goal(false, "Position feedback timed out; returning to dribble position");
    return;
  }

  if (state_ == State::HOLD_SHOOT) {
    if ((current_time - phase_start_time_).seconds() >= shoot_to_dribble_delay_sec_) {
      set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
    }
    return;
  }

  if ((current_time - phase_start_time_).seconds() > move_timeout_sec_) {
    target_position_rad_ = dribble_position_rad_;
    finish_goal(false, "Position move timed out; returning to dribble position");
  }
}

void DribblePositionController::set_target_position(double position_rad, State state)
{
  target_position_rad_ = position_rad;
  state_ = state;
  phase_start_time_ = now();
}

void DribblePositionController::finish_goal(bool succeeded, const std::string & message)
{
  if (!active_goal_) {
    return;
  }

  auto result = std::make_shared<DribblePosition::Result>();
  result->success = succeeded;
  result->message = message;
  if (succeeded) {
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
