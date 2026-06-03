#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TextSubscriber : public rclcpp::Node
{
  public:
    TextSubscriber() : Node("text_subscriber")
    {
        auto topic_callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "受信: '%s'", msg->data.c_str());
        };
        subscription_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, topic_callback);
    }

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TextSubscriber>());

    rclcpp::shutdown();
    return 0;
}