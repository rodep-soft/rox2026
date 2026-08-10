#include <memory>

#include "led_controller/led_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LedControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
