#include <memory>
#include <vector>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/u_int8.hpp"

class LimitSwNode : public rclcpp::Node
{
public:
  LimitSwNode()
  : Node("limit_sw_node")
  {
    this->declare_parameter<std::string>("can_receive_topic", "/CAN/can0/receive");
    this->declare_parameter<int>("target_can_id", 0x202);

    std::string topic_name = this->get_parameter("can_receive_topic").as_string();
    int target_id = this->get_parameter("target_can_id").as_int();
    target_can_id_ = static_cast<uint32_t>(target_id);

    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      topic_name, 10,
      std::bind(&LimitSwNode::canCallback, this, std::placeholders::_1));

    limit_sw_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/limit_sw", 10);

    RCLCPP_INFO(this->get_logger(), "Limit Switch Node has been started.");
    RCLCPP_INFO(
      this->get_logger(), "Listening on Topic: %s, Target CAN ID: 0x%X",
      topic_name.c_str(), target_can_id_);
  }

private:
  void canCallback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    if (msg->id == target_can_id_) {
      uint8_t raw_data = msg->data[0];
      uint8_t limit_status = raw_data & 0x07;

      auto output_msg = std_msgs::msg::UInt8();
      output_msg.data = limit_status;

      limit_sw_pub_->publish(output_msg);

      RCLCPP_INFO(this->get_logger(), "Published limit switch state: %d", limit_status);
    }
  }

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limit_sw_pub_;
  uint32_t target_can_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LimitSwNode>());
  rclcpp::shutdown();
  return 0;
}
