#include "game1_boundary_guard/game1_boundary_guard_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_controller::Game1BoundaryGuardNode>());
  rclcpp::shutdown();
  return 0;
}
