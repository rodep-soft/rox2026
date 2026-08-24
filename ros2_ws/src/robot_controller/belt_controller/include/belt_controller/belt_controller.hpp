#ifndef BELT_CONTROLLER__BELT_CONTROLLER_HPP_
#define BELT_CONTROLLER__BELT_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "actuator_msgs/msg/actuator_state_array.hpp"
#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "robot_msgs/msg/belt_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

class BeltControllerNode : public rclcpp::Node
{
public:
  BeltControllerNode();

private:
  static constexpr std::size_t num_levels = 4;

  void load_parameters();
  void belt_mode_callback(const robot_msgs::msg::BeltMode::SharedPtr msg);
  void belt_target_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void underbelt_target_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void upperbelt_target_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void vesc_state_callback(const actuator_msgs::msg::ActuatorStateArray::SharedPtr msg);
  void command_timer_callback();
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void publish_command();

  bool emergency_stop_active_{false};
  uint8_t belt_mode_{robot_msgs::msg::BeltMode::STOP};
  int direct_target_rpm_{0};
  int direct_underbelt_target_rpm_{0};
  int direct_upperbelt_target_rpm_{0};
  bool use_direct_target_rpm_{false};
  bool use_individual_direct_target_rpm_{false};

  float underbelt_measured_rpm_{0.0f};
  float upperbelt_measured_rpm_{0.0f};
  float last_underbelt_target_rpm_{0.0f};
  float last_upperbelt_target_rpm_{0.0f};

  // ── パラメータ ──────────────────────────────────────
  std::array<int, num_levels> underbelt_rpms_{3000, 3500, 4000, 4500};
  std::array<int, num_levels> upperbelt_rpms_{3000, 3500, 4000, 4500};
  int emergency_stop_period_ms_{50};
  int qos_depth_{1};
  uint16_t underbelt_logical_id_{11};
  uint16_t upperbelt_logical_id_{10};
  std::string target_array_topic_{"/vesc/target_array"};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr belt_target_rpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr underbelt_target_rpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr upperbelt_target_rpm_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorStateArray>::SharedPtr vesc_state_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr target_array_pub_;
  rclcpp::Publisher<robot_msgs::msg::BeltStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif  // BELT_CONTROLLER__BELT_CONTROLLER_HPP_
