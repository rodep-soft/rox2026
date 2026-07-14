#include <memory>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/u_int8.hpp"

class LimitSwNode : public rclcpp::Node
{
public:
    LimitSwNode() : Node("limit_sw_node")
    {
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "/CAN/can0/receive", 10,
            std::bind(&LimitSwNode::canCallback, this, std::placeholders::_1));
            
        limit_sw_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/limit_sw", 10);

        RCLCPP_INFO(this->get_logger(), "Limit Switch Node has been started.");
    }

private:
    void canCallback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        if (msg->id == 0x202) {
            uint8_t raw_data = msg->data[0];
            uint8_t limit_status = raw_data & 0x07;

            auto output_msg = std_msgs::msg::UInt8();
            output_msg.data = msg->data[0];

            limit_sw_pub_->publish(output_msg);
            
            RCLCPP_INFO(this->get_logger(), "Published limit switch state: %d", limit_status);
	}
	
    }
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limit_sw_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimitSwNode>());
    rclcpp::shutdown();
    return 0;
}
