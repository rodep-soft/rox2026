#ifndef ARM_POSITION_CONTROLLER__ARM_POSITION_CONTROLLER_HPP_
#define ARM_POSITION_CONTROLLER__ARM_POSITION_CONTROLLER_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class ArmPositionControllerNode : public rclcpp::Node
{
public:
  ArmPositionControllerNode();

private:
  enum class PositionMode : uint8_t
  {
    DRIBBLE = 0,
    OPEN = 1,
    FEED = 2,
  };

  void declare_parameters();
  void get_parameters();

  void position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_callback();

  const char * mode_name(PositionMode mode) const;

  double dribble_position_rad_{0.35};
  double open_position_rad_{-1.0};
  double feed_position_rad_{1.3};
  int command_period_ms_{20};
  int qos_depth_{1};

  PositionMode current_position_mode_{PositionMode::DRIBBLE};
  bool emergency_stop_active_{false};

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr position_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // ARM_POSITION_CONTROLLER__ARM_POSITION_CONTROLLER_HPP_
