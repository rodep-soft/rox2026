#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_state_array.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "can_msgs/msg/frame.hpp"
#include "edulite05_driver/edulite05_protocol.hpp"
#include "rclcpp/rclcpp.hpp"

namespace edulite05_driver
{
class Node : public rclcpp::Node
{
public:
  Node();

private:
  void get_parameters();

  // /socketcan_bridge/rxを受信し、対象モータの状態を更新して/edulite/stateへ配信する。
  void can_callback(can_msgs::msg::Frame::SharedPtr msg);

  // /edulite/targetの単体指令を、logical_idが一致するモータへ設定する。
  void target_callback(actuator_msgs::msg::ActuatorTarget::SharedPtr msg);

  // /edulite/targetsの一括指令を、各logical_idのモータへ設定する。
  void targets_callback(actuator_msgs::msg::ActuatorTargetArray::SharedPtr msg);

  // 初期化、通信断監視、通常CAN指令の送信を周期実行する。
  void update_callback();

  // 全モータの状態を/edulite/statesへ周期配信する。
  void states_callback();

  actuator_msgs::msg::ActuatorState make_state(const Protocol & motor) const;
  Protocol * find_can_id(uint8_t can_id);
  Protocol * find_logical_id(uint16_t logical_id);

  std::vector<Protocol> motors_;
  std::size_t init_index_ = 0;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr target_array_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorState>::SharedPtr state_pub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorStateArray>::SharedPtr state_array_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace edulite05_driver
