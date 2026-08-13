#include "game1_shooter/game1_auto_shooter_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::Game1AutoShooterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
