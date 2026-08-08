#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_state_array.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "actuator_msgs/srv/set_position.hpp"
#include "can_msgs/msg/frame.hpp"
#include "edulite05_driver/edulite05_protocol.hpp"
#include "rclcpp/rclcpp.hpp"

namespace edulite05_driver {
class Node : public rclcpp::Node {
public:
  Node();

private:
  void declare_and_load_parameters();
  void create_interfaces();

  // /socketcan_bridge/rx
  // の受信フレームから状態を更新し、/edulite/stateへ送信する。
  void can_frame_callback(can_msgs::msg::Frame::SharedPtr message);
  // /edulite/target の指令をlogical_idが一致するモーターへ設定する。
  void target_callback(actuator_msgs::msg::ActuatorTarget::SharedPtr message);
  // /edulite/target_array の各指令を対応するモーターへ設定する。
  void target_array_callback(
      actuator_msgs::msg::ActuatorTargetArray::SharedPtr message);
  // /edulite/set_positionを受け、PP/CSPの現在位置を指定値として校正する。
  // 未接続、速度制御、未知のlogical_idでは状態を変更せず失敗を返す。
  void set_position_callback(
      const std::shared_ptr<actuator_msgs::srv::SetPosition::Request> request,
      std::shared_ptr<actuator_msgs::srv::SetPosition::Response> response);

  void command_timer_callback();
  // 全モーターの状態を/edulite/state_arrayへ周期送信する。
  void state_timer_callback();

  actuator_msgs::msg::ActuatorState
  make_state_message(const Protocol &motor) const;
  Protocol *find_motor_by_can_id(uint8_t can_id);
  Protocol *find_motor_by_logical_id(uint16_t logical_id);

  std::vector<Protocol> motors_;
  std::size_t initialization_motor_index_ = 0;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_subscription_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTarget>::SharedPtr
      target_subscription_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr
      target_array_subscription_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorState>::SharedPtr
      state_publisher_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorStateArray>::SharedPtr
      state_array_publisher_;
  rclcpp::Service<actuator_msgs::srv::SetPosition>::SharedPtr
      set_position_service_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::string can_tx_topic_;
  std::string can_rx_topic_;
  std::string target_topic_;
  std::string target_array_topic_;
  std::string state_topic_;
  std::string state_array_topic_;
  std::string set_position_service_name_;
};
} // namespace edulite05_driver
