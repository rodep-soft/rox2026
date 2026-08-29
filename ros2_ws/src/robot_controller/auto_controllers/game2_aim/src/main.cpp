#include "game2_aim/game2_aim_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::Game2AimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
