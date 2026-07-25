#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joy_controller/joy_controller_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyControllerNode>());
  rclcpp::shutdown();
  return 0;
}
