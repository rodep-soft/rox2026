#include "game2_aim/pk_aim_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::PKAimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
