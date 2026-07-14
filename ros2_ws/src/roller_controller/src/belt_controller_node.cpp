#include "roller_controller/belt_controller_node.hpp"
#include "roller_controller/joy_button.hpp"

#include <algorithm>
#include <functional>
#include <memory>

namespace
{
constexpr int MinimumRpm = 0;
constexpr int MaximumRpm = 300;
}  // namespace

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  DeclareParameters();
  GetParameters();

  rpm_publisher_ = this->create_publisher<std_msgs::msg::Int16>(rpm_topic_, 10);
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&BeltControllerNode::JoyCallback, this, std::placeholders::_1));
}

void BeltControllerNode::DeclareParameters()
{
  this->declare_parameter<std::string>("rpm_topic", "/belt/rpm");
  this->declare_parameter<int>("enable_axis", 4);
  this->declare_parameter<double>("enable_axis_threshold", 0.5);
  this->declare_parameter<std::vector<int64_t>>("command_buttons", {2, 3, 0});
  this->declare_parameter<std::vector<int64_t>>("command_rpms", {0, 100, 200});
}

void BeltControllerNode::GetParameters()
{
  rpm_topic_ = this->get_parameter("rpm_topic").as_string();
  enable_axis_ = this->get_parameter("enable_axis").as_int();
  enable_axis_threshold_ = this->get_parameter("enable_axis_threshold").as_double();

  const auto configured_buttons = this->get_parameter("command_buttons").as_integer_array();
  const auto configured_rpms = this->get_parameter("command_rpms").as_integer_array();
  const size_t command_count = std::min(configured_buttons.size(), configured_rpms.size());

  if (configured_buttons.size() != configured_rpms.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "command_buttons and command_rpms have different lengths; using the first %zu pairs.",
      command_count);
  }

  command_buttons_.reserve(command_count);
  command_rpms_.reserve(command_count);
  for (size_t index = 0; index < command_count; ++index) {
    command_buttons_.push_back(static_cast<int>(configured_buttons[index]));
    const int clamped_rpm = std::clamp(
      static_cast<int>(configured_rpms[index]), MinimumRpm, MaximumRpm);
    command_rpms_.push_back(static_cast<int16_t>(clamped_rpm));
  }
}

void BeltControllerNode::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std_msgs::msg::Int16 rpm_msg;
  rpm_msg.data = SelectRpm(*msg);
  rpm_publisher_->publish(rpm_msg);
}

int16_t BeltControllerNode::SelectRpm(const sensor_msgs::msg::Joy & msg)
{
  if (
    !roller_controller::IsAxisPressed(*this, msg, enable_axis_, enable_axis_threshold_))
  {
    return 0;
  }

  for (size_t index = 0; index < command_buttons_.size(); ++index) {
    if (roller_controller::IsButtonPressed(*this, msg, command_buttons_[index])) {
      return command_rpms_[index];
    }
  }
  return 0;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeltControllerNode>());
  rclcpp::shutdown();
  return 0;
}
