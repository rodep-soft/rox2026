#include <memory>

#include "joy_controller/cmd_vel_selector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<joy_controller::CmdVelSelectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
