#include "dribble_position_controller/dribble_position_controller.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <utility>

// DribblePosition Actionは、dribble機構の位置移動を管理する。
// GoalはDRIBBLEまたはSHOOTだけを受け付け、それ以外のcommandは拒否する。
// DRIBBLE指令ではドリブル位置へ移動し、到達したらActionを成功完了する。
// SHOOT指令では取り込み位置へ移動し、到達後にシュート位置へ進む。
// 各移動はhardware_driverからの実位置feedbackを見て、目標位置との差が
// position_tolerance_rad(許容範囲内のrad)以内になったタイミングで次の状態へ進める。
// 新しいgoalを受けた場合は、実行中のgoalを中断して新しい移動を開始する。

DribblePositionController::DribblePositionController()
: Node("dribble_position_controller")
{
  declare_parameters();
  get_parameters();

  if (return_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "return_timeout_sec must be greater than zero. Using the default value of 3.0 s.");
    return_timeout_sec_ = 3.0;
  }

  position_command_pub_ = create_publisher<std_msgs::msg::Float32>(
    dribble_position_command_topic_, rclcpp::QoS(qos_depth_));
  position_feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
    dribble_position_feedback_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&DribblePositionController::position_feedback_callback, this, std::placeholders::_1));
  action_server_ = rclcpp_action::create_server<DribblePosition>(
    this, dribble_position_action_,
    std::bind(
      &DribblePositionController::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&DribblePositionController::handle_cancel, this, std::placeholders::_1),
    std::bind(&DribblePositionController::handle_accepted, this, std::placeholders::_1));
  return_timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DribblePositionController::return_timeout_callback, this));

  publish_target_position(dribble_position_rad_);
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
  declare_parameter<double>("return_timeout_sec", 3.0);
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
  get_parameter("return_timeout_sec", return_timeout_sec_);
}

rclcpp_action::GoalResponse DribblePositionController::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const DribblePosition::Goal> goal)
{
  // Actionで定義した2種類の位置指令だけを受け付ける。
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
    RCLCPP_INFO(get_logger(), "Dribble position goal cancelled. Returning to dribble position.");
    start_return_to_dribble(
      Completion::CANCELED,
      "Goal cancelled after returning to dribble position");
  } else if (goal_handle == pending_goal_) {
    RCLCPP_INFO(get_logger(), "Pending dribble position goal cancelled before execution.");
    auto result = std::make_shared<DribblePosition::Result>();
    result->success = false;
    result->message = "Goal cancelled before execution";
    pending_goal_->canceled(result);
    pending_goal_.reset();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DribblePositionController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  if (active_goal_) {
    // 実行中のgoalはDRIBBLE位置への復帰を確認してから中断し、新しいgoalを開始する。
    if (pending_goal_) {
      RCLCPP_WARN(get_logger(), "Rejected a dribble position goal while another goal is pending.");
      auto result = std::make_shared<DribblePosition::Result>();
      result->success = false;
      result->message = "Rejected because another goal is pending";
      goal_handle->abort(result);
      return;
    }
    RCLCPP_WARN(get_logger(), "Dribble position goal preempted. Returning to dribble position.");
    pending_goal_ = goal_handle;
    start_return_to_dribble(Completion::ABORTED, "Preempted by a new goal");
    return;
  }

  start_goal(goal_handle);
}

void DribblePositionController::start_goal(const std::shared_ptr<GoalHandle> goal_handle)
{
  active_goal_ = goal_handle;
  if (goal_handle->get_goal()->command == DribblePosition::Goal::DRIBBLE) {
    state_ = State::DRIBBLE;
    publish_target_position(dribble_position_rad_);
  } else {
    // SHOOTは取り込み位置へ移動してからシュート位置へ進む。
    state_ = State::INTAKE;
    publish_target_position(intake_position_rad_);
  }
}

void DribblePositionController::start_return_to_dribble(
  Completion completion, const std::string & message)
{
  return_completion_ = completion;
  return_start_time_ = now();
  state_ = State::RETURN_TO_DRIBBLE;
  publish_target_position(dribble_position_rad_);
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

void DribblePositionController::position_feedback_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  current_position_rad_ = msg->data;
  if (!active_goal_) {
    return;
  }

  // 実行中の移動状態をfeedbackとして返す。
  auto feedback = std::make_shared<DribblePosition::Feedback>();
  feedback->state = static_cast<uint8_t>(state_);
  feedback->target_position_rad = static_cast<float>(target_position_rad_);
  feedback->current_position_rad = static_cast<float>(current_position_rad_);
  active_goal_->publish_feedback(feedback);

  if (std::abs(current_position_rad_ - target_position_rad_) > position_tolerance_rad_) {
    return;
  }

  // 現在の目標位置に到達したら、次の状態へ進める。
  switch (state_) {
    case State::DRIBBLE:
      finish_goal(Completion::SUCCEEDED, "Reached dribble position");
      break;
    case State::INTAKE:
      state_ = State::SHOOT;
      publish_target_position(shoot_position_rad_);
      break;
    case State::SHOOT:
      finish_goal(Completion::SUCCEEDED, "Reached shoot position");
      break;
    case State::RETURN_TO_DRIBBLE: {
        RCLCPP_INFO(get_logger(), "Returned to dribble position.");
        const auto completion = return_completion_;
        finish_goal(
          completion,
          completion == Completion::CANCELED ?
          "Goal cancelled after returning to dribble position" :
          "Goal aborted after returning to dribble position");
        if (pending_goal_) {
          auto pending_goal = pending_goal_;
          pending_goal_.reset();
          start_goal(pending_goal);
        }
      }
      break;
  }
}

void DribblePositionController::return_timeout_callback()
{
  if (!active_goal_ || state_ != State::RETURN_TO_DRIBBLE) {
    return;
  }
  if ((now() - return_start_time_).seconds() <= return_timeout_sec_) {
    return;
  }

  RCLCPP_ERROR(
    get_logger(),
    "Timed out while returning to dribble position. Aborting active and pending goals.");
  finish_goal(Completion::ABORTED, "Timed out while returning to dribble position");
  abort_pending_goal("Timed out while waiting for return to dribble position");
}

void DribblePositionController::publish_target_position(double position_rad)
{
  target_position_rad_ = position_rad;
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(position_rad);
  position_command_pub_->publish(command);
}

void DribblePositionController::finish_goal(Completion completion, const std::string & message)
{
  if (!active_goal_) {
    return;
  }
  // Actionの結果を返し、実行中のgoalを終了状態にする。
  auto result = std::make_shared<DribblePosition::Result>();
  result->success = completion == Completion::SUCCEEDED;
  result->message = message;
  switch (completion) {
    case Completion::SUCCEEDED:
      active_goal_->succeed(result);
      break;
    case Completion::ABORTED:
      active_goal_->abort(result);
      break;
    case Completion::CANCELED:
      active_goal_->canceled(result);
      break;
  }
  active_goal_.reset();
}

void DribblePositionController::abort_pending_goal(const std::string & message)
{
  if (!pending_goal_) {
    return;
  }
  auto result = std::make_shared<DribblePosition::Result>();
  result->success = false;
  result->message = message;
  pending_goal_->abort(result);
  pending_goal_.reset();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribblePositionController>());
  rclcpp::shutdown();
  return 0;
}
