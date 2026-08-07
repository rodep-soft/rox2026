#ifndef EDULITE05_DRIVER__EDULITE05_PROTOCOL_HPP_
#define EDULITE05_DRIVER__EDULITE05_PROTOCOL_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "can_msgs/msg/frame.hpp"

namespace edulite05_driver
{

enum class Mode : uint8_t
{
  PP = 1,
  VELOCITY = 2,
  CSP = 5
};

enum class MotorState : uint8_t
{
  OFFLINE,
  INITIALIZING,
  READY,
  ERROR
};

struct MotorConfig
{
  std::string name;
  uint16_t logical_id = 0;
  uint8_t can_id = 0;
  Mode mode = Mode::VELOCITY;

  float current_limit = 11.0f;
  float acceleration = 150.0f;
  float speed_limit = 50.0f;
    // CAN指令送信周期
  uint32_t command_period_ms = 10;
  // 上位ノードからの指令タイムアウト
  uint32_t target_timeout_ms = 200;
  // モータから応答がなくなったときのタイムアウト
  uint32_t feedback_timeout_ms = 500;
};

struct MotorFeedback
{
  float position = 0.0f;
  float velocity = 0.0f;
  float effort = 0.0f;
  float temperature = 0.0f;
  uint32_t fault_code = 0;
};

class Protocol
{
public:
  explicit Protocol(const MotorConfig & config);

  const std::string & get_name() const { return config_.name; }
  uint16_t get_logical_id() const { return config_.logical_id; }
  uint8_t get_can_id() const { return config_.can_id; }

  MotorState get_state() const { return state_; }
  bool is_connected() const { return connected_; }
  bool is_enabled() const { return enabled_; }

  const MotorFeedback & get_feedback() const { return feedback_; }

  void set_target(float target);

  // 初期化状態機械を1ステップ進める
  std::optional<can_msgs::msg::Frame> initialization_frame();

  // CAN受信
  void receive(const can_msgs::msg::Frame & msg,const rclcpp::Time & now);

  // READY時の通常指令
  std::optional<can_msgs::msg::Frame> target_frame() const;

  // 通信断監視
  void watchdog(const rclcpp::Time & now);

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  enum class InitStep
  {
    WRITE_MODE,
    READ_MODE,
    WAIT_MODE,

    WRITE_PARAM,
    READ_PARAM,
    WAIT_PARAM,

    ENABLE,
    WAIT_ENABLE,

    READY,
    ERROR
  };

  struct Parameter
  {
    uint16_t index;
    float value;
  };


  MotorConfig config_;
  MotorFeedback feedback_;

  MotorState state_ = MotorState::INITIALIZING;
  InitStep step_ = InitStep::WRITE_MODE;

  std::vector<Parameter> parameters_;
  size_t parameter_index_ = 0;

  float target_ = 0.0f;

  bool connected_ = false;
  bool enabled_ = false;

  int retry_count_ = 0;

  rclcpp::Time last_request_;
  rclcpp::Time last_feedback_;

  void build_parameters();
  void retry();
  void restart();

  void receive_feedback(const can_msgs::msg::Frame & msg, const rclcpp::Time & now);
  void receive_parameter(const can_msgs::msg::Frame & msg,const rclcpp::Time & now);

  static can_msgs::msg::Frame write_u8(uint8_t motor_id, uint16_t index, uint8_t value);
  static can_msgs::msg::Frame write_float(uint8_t motor_id, uint16_t index, float value);
  static can_msgs::msg::Frame read(uint8_t motor_id, uint16_t index);
  static can_msgs::msg::Frame enable(uint8_t motor_id);
};

}  // namespace edulite05_driver