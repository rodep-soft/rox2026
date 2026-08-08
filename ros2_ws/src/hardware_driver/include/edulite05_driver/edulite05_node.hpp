#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_state_array.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "actuator_msgs/srv/set_position.hpp"

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
  /// @brief yamlから各種情報の取得
  void get_parameters();

  /// @brief can_msgs::msg::Frameを受信し，対象モータの状態を更新して/edulite/stateへ配信
  /// @param msg 受信したcanFrame 
  void can_callback(can_msgs::msg::Frame::SharedPtr msg);

  /// @brief 単体の目標値指令をlogical_idが一致するモータへ設定
  /// @param msg actuator_msgs::msg::ActuatorTargetの目標値指令
  void target_callback(actuator_msgs::msg::ActuatorTarget::SharedPtr msg);

  /// @brief 複数の目標値指令を受信し，各モータへ設定
  /// @param msg actuator_msgs::msg::ActuatorTargetArrayの目標値指令
  void target_array_callback(actuator_msgs::msg::ActuatorTargetArray::SharedPtr msg);

  /// @brief 初期化，通信断監視，通常CAN指令の送信を周期実行
  void update_callback();

  /// @brief actuator_msgs::msg::ActuatorStateArrayの状態配信
  void state_array_callback();

  /// @brief モータの状態をactuator_msgs::msg::ActuatorStateに変換
  /// @param motor モータのProtocol
  /// @return 変換された状態
  actuator_msgs::msg::ActuatorState make_state(const Protocol & motor) const;
  
  /// @brief CAN IDからモータを検索
  /// @param can_id can_id
  /// @return モータのポインタ
  Protocol * find_can_id(uint8_t can_id);

  /// @brief logical_idからモータを検索
  /// @param logical_id logical_id
  /// @return モータのポインタ
  Protocol * find_logical_id(uint16_t logical_id);

  void set_position_callback(const std::shared_ptr<actuator_msgs::srv::SetPosition::Request> request,
  std::shared_ptr<actuator_msgs::srv::SetPosition::Response> response);

  std::vector<Protocol> motors_;
  std::size_t init_index_ = 0;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr target_array_sub_;

  rclcpp::Publisher<actuator_msgs::msg::ActuatorState>::SharedPtr state_pub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorStateArray>::SharedPtr state_array_pub_;

  rclcpp::Service<actuator_msgs::srv::SetPosition>::SharedPtr set_position_srv_;
  
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::string can_pub_topic;
  std::string can_sub_topic;
  std::string target_topic;
  std::string target_array_topic;
  std::string state_topic;
  std::string state_array_topic;
};

}  // namespace edulite05_driver
