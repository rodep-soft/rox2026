#include "game2_shooter/game2_auto_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::Game2AutoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
