#include <memory>

#include "arm_position_controller/arm_position_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmPositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
