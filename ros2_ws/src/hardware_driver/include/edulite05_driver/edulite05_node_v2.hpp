#ifndef EDULITE05_DRIVER__EDULITE05_NODE_HPP_
#define EDULITE05_DRIVER__EDULITE05_NODE_HPP_

#pragma once

#include <memory>
#include <vector>

#include "actuator_msgs/msg/actuator_state_array.hpp"
#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

#include "edulite05_driver/protocol.hpp"

class Ed05DriverNode : public rclcpp::Node
{

public:
 Ed05DriverNode();

private:
  void get_parameters();

  void can_callback(can_msgs::msg::Frame::SharedPtr msg);
  
  void target_callback(actuator_msgs::msg::ActuatorTarget::SharedPtr msg);
  void targets_callback(actuator_msgs::msg::ActuatorTargetArray::SharedPtr msg);

  void update_callback();
  void states_callback();

  void set_target(uint16_t logical_id,float target);

  actuator_msgs::msg::ActuatorState make_state(const Protocol & motor) const;
  
  Protocol * find_can_id(uint8_t can_id);
  Protocol * find_logical_id(uint16_t logical_id);

  std::vector<Protocol> motors_;

  size_t init_index_ = 0;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;


  rclcpp::Subscription<actuator_msgs::msg::ActuatorTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr target_array_sub_;

  rclcpp::Publisher<actuator_msgs::msg::ActuatorState>::SharedPtr state_pub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorStateArray>::SharedPtr state_array_pub_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

#endif  // EDULITE05_DRIVER__EDULITE05_NODE_HPP_
