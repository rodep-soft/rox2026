#include "spring_controller/spring_edulite_controller.hpp"

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

SpringEduliteController::SpringEduliteController()
: Node("spring_controller_node")
{
  const auto robot_command_topic = declare_parameter<std::string>(
    "robot_command_topic", "/robot_command");
  const auto limit_switch_topic = declare_parameter<std::string>(
    "limit_switch_topic", "/limit_switches");
  const auto spring_velocity_command_topic = declare_parameter<std::string>(
    "spring_velocity_command_topic", "/spring_edulite_speed");
  limit_switch_index_ = declare_parameter<int>("limit_switch_index", 0);
  loading_velocity_rad_s_ = declare_parameter<double>("loading_velocity_rad_s", -5.0);
  fire_velocity_rad_s_ = declare_parameter<double>("fire_velocity_rad_s", -20.0);
  fire_duration_sec_ = declare_parameter<double>("fire_duration_sec", 5.0);

  if (limit_switch_index_ < 0) {
    RCLCPP_ERROR(get_logger(), "limit_switch_index must be zero or greater");
    is_configuration_valid_ = false;
  }
  if (fire_duration_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "fire_duration_sec must be greater than zero");
    is_configuration_valid_ = false;
  }

  robot_command_sub_ = create_subscription<robot_controller::msg::RobotCommand>(
    robot_command_topic, 10,
    std::bind(&SpringEduliteController::robot_command_callback, this, std::placeholders::_1));
  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    limit_switch_topic, 10,
    std::bind(&SpringEduliteController::limit_switch_callback, this, std::placeholders::_1));
  spring_velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
    spring_velocity_command_topic, 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&SpringEduliteController::timer_callback, this));
}

void SpringEduliteController::robot_command_callback(
  const robot_controller::msg::RobotCommand::SharedPtr msg)
{
  if (
    now_state_ == State::READY && is_loaded_ &&
    msg->spring_is_fire && !previous_fire_request_)
  {
    fire_pending_ = true;
  }
  previous_fire_request_ = msg->spring_is_fire;
}

void SpringEduliteController::limit_switch_callback(
  const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  const auto index = static_cast<std::size_t>(limit_switch_index_);
  if (limit_switch_index_ < 0 || index >= msg->data.size()) {
    RCLCPP_ERROR(
      get_logger(), "limit_switch_index %d is outside the received array of %zu elements",
      limit_switch_index_, msg->data.size());
    return;
  }
  is_loaded_ = msg->data[index] != 0;
}

void SpringEduliteController::start_fire()
{
  now_state_ = State::FIRE;
  fire_start_time_ = now();
  fire_pending_ = false;
}

void SpringEduliteController::timer_callback()
{
  std_msgs::msg::Float32 velocity_command;

  if (!is_configuration_valid_) {
    velocity_command.data = 0.0F;
    spring_velocity_pub_->publish(velocity_command);
    return;
  }

  switch (now_state_) {
    case State::LOAD:
      if (is_loaded_) {
        now_state_ = State::READY;
        velocity_command.data = 0.0F;
      } else {
        velocity_command.data = static_cast<float>(loading_velocity_rad_s_);
      }
      break;

    case State::READY:
      if (fire_pending_ && is_loaded_) {
        start_fire();
        velocity_command.data = static_cast<float>(fire_velocity_rad_s_);
      } else if (!is_loaded_) {
        now_state_ = State::LOAD;
        velocity_command.data = static_cast<float>(loading_velocity_rad_s_);
      } else {
        velocity_command.data = 0.0F;
      }
      break;

    case State::FIRE:
      velocity_command.data = static_cast<float>(fire_velocity_rad_s_);
      if ((now() - fire_start_time_).seconds() >= fire_duration_sec_) {
        now_state_ = State::LOAD;
      }
      break;
  }

  spring_velocity_pub_->publish(velocity_command);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringEduliteController>());
  rclcpp::shutdown();
  return 0;
}
