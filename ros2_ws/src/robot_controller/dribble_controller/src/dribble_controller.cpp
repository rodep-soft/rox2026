#include "dribble_controller/dribble_controller.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
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
  if (
    on_rpm_ < std::numeric_limits<int16_t>::min() ||
    on_rpm_ > std::numeric_limits<int16_t>::max())
  {
    RCLCPP_ERROR(get_logger(), "on_rpm must fit in std_msgs/msg/Int16");
    is_configuration_valid_ = false;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using the default value of 1.");
    qos_depth_ = 1;
  }

  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    dribble_enabled_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&DribbleController::dribble_enabled_callback, this, std::placeholders::_1));
  rpm_pub_ = create_publisher<std_msgs::msg::Int16>(
    dribble_rpm_topic_, rclcpp::QoS(qos_depth_));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribbleController::timer_callback, this));
}

void DribbleController::declare_parameters()
{
  declare_parameter<std::string>("dribble_enabled_topic", "/dribble/enabled");
  declare_parameter<std::string>("dribble_rpm_topic", "/stm32/dribble/target/rpm");
  declare_parameter<int>("on_rpm", 600);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void DribbleController::get_parameters()
{
  get_parameter("dribble_enabled_topic", dribble_enabled_topic_);
  get_parameter("dribble_rpm_topic", dribble_rpm_topic_);
  get_parameter("on_rpm", on_rpm_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void DribbleController::dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  dribble_enabled_ = msg->data;
}

void DribbleController::timer_callback()
{
  // onなら目標RPM、offまたは設定不正なら0 RPMを送る。
  const int target_rpm = (is_configuration_valid_ && dribble_enabled_) ? on_rpm_ : 0;

  std_msgs::msg::Int16 rpm_command;
  rpm_command.data = static_cast<int16_t>(target_rpm);
  rpm_pub_->publish(rpm_command);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribbleController>());
  rclcpp::shutdown();
  return 0;
}
