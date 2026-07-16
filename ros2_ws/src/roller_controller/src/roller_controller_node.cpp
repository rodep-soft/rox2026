#include "roller_controller/roller_controller_node.hpp"
#include "roller_controller/joy_button.hpp"

#include <functional>
#include <memory>

RollerControllerNode::RollerControllerNode()
: Node("roller_controller_node")
{
  DeclareParameters();
  GetParameters();
  current_rpm_ = stop_rpm_;

  rpm_publisher_ = this->create_publisher<std_msgs::msg::Int16>(rpm_topic_, 10);
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy_second", 10,
    std::bind(&RollerControllerNode::JoyCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "RPM controller started. rpm_topic=%s", rpm_topic_.c_str());
}

void RollerControllerNode::DeclareParameters()
{
  this->declare_parameter<std::string>("rpm_topic", "/roller/rpm");
  this->declare_parameter<int>("enable_axis", 3);
  this->declare_parameter<double>("enable_axis_threshold", 0.5);
  this->declare_parameter<int>("stop_button", 2);
  this->declare_parameter<int>("high_button", 0);
  this->declare_parameter<int>("low_button", 3);
  this->declare_parameter<int>("stop_rpm", 0);
  this->declare_parameter<int>("high_rpm", 4000);
  this->declare_parameter<int>("low_rpm", 3000);
}

void RollerControllerNode::GetParameters()
{
  rpm_topic_ = this->get_parameter("rpm_topic").as_string();
  enable_axis_ = this->get_parameter("enable_axis").as_int();
  enable_axis_threshold_ = this->get_parameter("enable_axis_threshold").as_double();

  stop_button_ = this->get_parameter("stop_button").as_int();
  high_button_ = this->get_parameter("high_button").as_int();
  low_button_ = this->get_parameter("low_button").as_int();
  stop_rpm_ = static_cast<int16_t>(this->get_parameter("stop_rpm").as_int());
  high_rpm_ = static_cast<int16_t>(this->get_parameter("high_rpm").as_int());
  low_rpm_ = static_cast<int16_t>(this->get_parameter("low_rpm").as_int());
}

void RollerControllerNode::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std_msgs::msg::Int16 rpm_msg;
  current_rpm_ = SelectRpm(*msg);
  rpm_msg.data = current_rpm_;
  rpm_publisher_->publish(rpm_msg);
}

int16_t RollerControllerNode::SelectRpm(const sensor_msgs::msg::Joy & msg)
{
  if (
    !roller_controller::IsAxisPressed(*this, msg, enable_axis_, enable_axis_threshold_))
  {
    return current_rpm_;
  }

  if (roller_controller::IsButtonPressed(*this, msg, stop_button_)) {
    return stop_rpm_;
  }
  if (roller_controller::IsButtonPressed(*this, msg, high_button_)) {
    return high_rpm_;
  }
  if (roller_controller::IsButtonPressed(*this, msg, low_button_)) {
    return low_rpm_;
  }

  return current_rpm_;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RollerControllerNode>());
  rclcpp::shutdown();
  return 0;
}
