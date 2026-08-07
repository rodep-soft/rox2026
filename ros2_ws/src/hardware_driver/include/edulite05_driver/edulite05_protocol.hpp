#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "can_msgs/msg/frame.hpp"

namespace edulite05_driver
{
  constexpr uint8_t HOST_ID = 0xFD;
  constexpr uint8_t TYPE_FEEDBACK = 0x02;
  constexpr uint8_t TYPE_ENABLE = 0x03;
  constexpr uint8_t TYPE_READ = 0x11;
  constexpr uint8_t TYPE_WRITE = 0x12;

  // Parameter index
  constexpr uint16_t RUN_MODE = 0x7005;
  constexpr uint16_t SPEED_REF = 0x700A;

  constexpr uint16_t POSITION_REF = 0x7016;
  constexpr uint16_t LIMIT_SPEED = 0x7017;
  constexpr uint16_t LIMIT_CURRENT = 0x7018;

  constexpr uint16_t ACCELERATION = 0x7022;
  constexpr uint16_t PP_SPEED = 0x7024;
  constexpr uint16_t PP_ACCELERATION = 0x7025;

  constexpr int MAX_RETRY = 3;
  constexpr auto WRITE_WAIT = std::chrono::milliseconds(2);
  constexpr auto RESPONSE_TIMEOUT = std::chrono::milliseconds(100);
  constexpr auto ERROR_RETRY_PERIOD = std::chrono::milliseconds(1000);
  constexpr float PI = 3.14159265358979323846f;


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
  void receive(const can_msgs::msg::Frame & msg);

  // READY時の通常指令
  std::optional<can_msgs::msg::Frame> target_frame() const;

  // 通信断監視
  void watchdog();

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

  TimePoint last_feedback_time_{};
  TimePoint last_request_time_{};
  TimePoint last_target_time_{};
  TimePoint last_command_time_{};


  void build_parameters();
  void retry();
  void restart();

  void receive_feedback(const can_msgs::msg::Frame & msg);
  void receive_parameter(const can_msgs::msg::Frame & msg);

  static can_msgs::msg::Frame write_u8(uint8_t motor_id, uint16_t index, uint8_t value);
  static can_msgs::msg::Frame write_float(uint8_t motor_id, uint16_t index, float value);
  static can_msgs::msg::Frame read(uint8_t motor_id, uint16_t index);
  static can_msgs::msg::Frame enable(uint8_t motor_id);
};

}  // namespace edulite05_driver