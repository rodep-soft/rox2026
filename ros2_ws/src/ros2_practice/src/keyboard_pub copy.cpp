#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class KeyboardPublisher : public rclcpp::Node
{
  public:
    KeyboardPublisher() : Node("keyboard_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        auto timer_callback = [this]() -> void {
            std::string input;

            std::cout << "入力してください: ";
            std::getline(std::cin, input);

            auto message = std_msgs::msg::String();
            message.data = input;

            RCLCPP_INFO(this->get_logger(), "送信: '%s'", message.data.c_str());

            this->publisher_->publish(message);
        };

        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<KeyboardPublisher>());

    rclcpp::shutdown();
    return 0;
}