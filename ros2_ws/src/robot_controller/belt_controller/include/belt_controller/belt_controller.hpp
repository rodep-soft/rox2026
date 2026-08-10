#ifndef BELT_CONTROLLER__BELT_CONTROLLER_HPP_
#define BELT_CONTROLLER__BELT_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class BeltControllerNode : public rclcpp::Node
{
public:
  BeltControllerNode();

private:
  static constexpr std::size_t num_levels = 4;

  enum class BeltMode : uint8_t
  {
    STOP = 0,
    LEVEL_1 = 1,
    LEVEL_2 = 2,
    LEVEL_3 = 3,
    LEVEL_4 = 4,
  };

  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void belt_target_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_timer_callback();
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void publish_command();

  bool emergency_stop_active_{false};
  BeltMode belt_mode_{BeltMode::STOP};
  int direct_target_rpm_{0};
  bool use_direct_target_rpm_{false};

  std::array<int, num_levels> underbelt_rpms_{3000, 3500, 4000, 4500};
  std::array<int, num_levels> upperbelt_rpms_{3000, 3500, 4000, 4500};
  uint16_t underbelt_logical_id_{11};
  uint16_t upperbelt_logical_id_{10};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr belt_target_rpm_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr target_array_pub_;
  rclcpp::TimerBase::SharedPtr emergency_stop_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif  // BELT_CONTROLLER__BELT_CONTROLLER_HPP_
