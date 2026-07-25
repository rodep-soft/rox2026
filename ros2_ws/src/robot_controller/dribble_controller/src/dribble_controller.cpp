#include "dribble_controller/dribble_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

DribbleController::DribbleController()
: Node("dribble_controller_node")
{
  declare_parameters();
  get_parameters();

  if (command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "command_period_ms must be greater than zero");
    is_configuration_valid_ = false;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using the default value of 1.");
    qos_depth_ = 1;
  }

  dribble_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    dribble_mode_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&DribbleController::dribble_mode_callback, this, std::placeholders::_1));
  rpm_pub_ = create_publisher<std_msgs::msg::Int16>(dribble_rpm_topic_, rclcpp::QoS(qos_depth_));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribbleController::timer_callback, this));
}

void DribbleController::declare_parameters()
{
  declare_parameter<std::string>("dribble_mode_topic", "/dribble/mode");
  declare_parameter<std::string>("dribble_rpm_topic", "/dribble/target/rpm");
  declare_parameter<double>("low_rpm", 300.0);
  declare_parameter<double>("high_rpm", 600.0);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void DribbleController::get_parameters()
{
  get_parameter("dribble_mode_topic", dribble_mode_topic_);
  get_parameter("dribble_rpm_topic", dribble_rpm_topic_);
  get_parameter("low_rpm", low_rpm_);
  get_parameter("high_rpm", high_rpm_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void DribbleController::dribble_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  dribble_mode_ = msg->data;
}

double DribbleController::target_rpm_from_mode(uint8_t mode)
{
  switch (mode) {
    case stop_mode_:
      return 0.0;
    case high_mode_:
      return high_rpm_;
    case low_mode_:
      return low_rpm_;
    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Unsupported dribble_mode %u. Stopping dribble.", mode);
      return 0.0;
  }
}

void DribbleController::timer_callback()
{
  current_rpm_ = target_rpm_from_mode(dribble_mode_);

  if (!is_configuration_valid_) {
    current_rpm_ = 0.0;
  }

  std_msgs::msg::Int16 rpm_command;
  rpm_command.data = static_cast<int16_t>(std::round(current_rpm_));
  rpm_pub_->publish(rpm_command);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribbleController>());
  rclcpp::shutdown();
  return 0;
}
