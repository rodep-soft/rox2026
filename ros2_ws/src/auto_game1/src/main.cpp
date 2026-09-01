#include "auto_game1/auto_game1_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_game1::AutoGame1Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
