#include "mecanum_controller/mecanum_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumControllerNode>());
  rclcpp::shutdown();
  return 0;
}
