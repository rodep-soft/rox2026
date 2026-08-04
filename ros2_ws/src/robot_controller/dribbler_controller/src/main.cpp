#include <memory>

#include "dribbler_controller/dribbler_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribblerControllerNode>());
  rclcpp::shutdown();
  return 0;
}
