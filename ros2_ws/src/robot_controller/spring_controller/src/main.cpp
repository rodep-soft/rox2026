#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "spring_controller/spring_edulite_controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpringEduliteController>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}