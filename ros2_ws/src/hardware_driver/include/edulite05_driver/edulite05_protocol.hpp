#pragma once

#include <chrono>
#include <cstddef>
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

constexpr uint8_t RESET_STATUS_MODE = 0x00;
constexpr uint8_t RUN_STATUS_MODE = 0x02;


constexpr uint16_t RUN_MODE = 0x7005;
constexpr uint16_t SPEED_REF = 0x700A;
constexpr uint16_t POSITION_REF = 0x7016;
constexpr uint16_t LIMIT_SPEED = 0x7017;
constexpr uint16_t LIMIT_CURRENT = 0x7018;
constexpr uint16_t ACCELERATION = 0x7022;
constexpr uint16_t PP_SPEED = 0x7024;
constexpr uint16_t PP_ACCELERATION = 0x7025;

constexpr int MAX_RETRY = 10;
constexpr auto WRITE_WAIT = std::chrono::milliseconds(2);
constexpr auto RESPONSE_TIMEOUT = std::chrono::milliseconds(100);
constexpr auto ERROR_RETRY_PERIOD = std::chrono::milliseconds(200);
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

  float position_offset = 0.0f;
  float position_min = -1000.0f;
  float position_max = 1000.0f;
  bool require_homing = false;

  uint32_t command_period_ms = 10;
  uint32_t target_timeout_ms = 200;
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

  const std::string & get_name() const {return config_.name;}
  uint16_t get_logical_id() const {return config_.logical_id;}
  uint8_t get_can_id() const {return config_.can_id;}
  MotorState get_state() const {return state_;}
  bool is_homed() const{return homed_;}
  bool is_connected() const {return connected_;}
  bool is_configured() const {return configured_;}
  bool is_enabled() const {return enabled_;}
  const MotorFeedback & get_feedback() const {return feedback_;}

  /// @brief 目標値を設定
  /// @param target 目標値（VELOCITYモードでは速度[rad/s]，PP/CSPモードでは位置[rad]）
  void set_target(float target);

  bool set_position_offset(float offset);

  /// @brief 今回の呼び出しで初期化の際に送るフレームの取得
  /// @return 初期化フレーム
  std::optional<can_msgs::msg::Frame> create_initialization_frame();

  /// @brief CANフレームを受信
  /// @param msg 受信したCANフレーム
  /// @return 処理結果 true: 値を受信できた，false: 受信できなかった
  bool receive(const can_msgs::msg::Frame & msg);

  /// @brief 目標値の送信が必要かどうか
  /// @return true: 送信が必要，false: 送信不要
  std::optional<can_msgs::msg::Frame> create_target_frame();
  void watchdog();

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  enum class InitStep
  {
    WRITE_ITEM,
    WAIT_WRITE,
    READ_ITEM,
    WAIT_READ,
    ENABLE,
    WAIT_ENABLE,
    READY,
    ERROR
  };

  struct InitItem
  {
    uint16_t index;
    float value;
    bool is_u8;
  };

  MotorConfig config_;
  MotorFeedback feedback_;
  MotorState state_ = MotorState::INITIALIZING;
  InitStep init_step_ = InitStep::WRITE_ITEM;
  std::vector<InitItem> init_items_;
  std::size_t init_index_ = 0;
  float target_ = 0.0f;
  bool target_received_ = false;
  bool connected_ = false;
  bool configured_ = false;
  bool enabled_ = false;
  int retry_count_ = 0;

  float raw_position_ = 0.0f;
  float position_offset_ = 0.0f;
  bool homed_ = false;

  TimePoint last_rx_time_{};
  TimePoint last_request_time_{};
  TimePoint last_target_time_{};
  TimePoint last_command_time_{};
  TimePoint error_time_{};

  /// @brief 初期化のリトライ
  void retry_initialization();
  /// @brief モータの初期化からやり直すことを指示
  /// @param clear_target やり直すモータの目標値を破棄するかどうか
  void restart(bool clear_target);

  /// @brief フィードバックを処理
  /// @param msg 受信したCANフレーム
  void process_feedback(const can_msgs::msg::Frame & msg);

  /// @brief パラメータ応答を処理
  /// @param msg 受信したCANフレーム
  void process_parameter_response(const can_msgs::msg::Frame & msg);

  static can_msgs::msg::Frame make_base_frame(uint8_t type, uint8_t motor_id);
  static can_msgs::msg::Frame create_write_u8_frame(uint8_t motor_id, uint16_t index, uint8_t value);
  static can_msgs::msg::Frame create_write_float_frame(uint8_t motor_id, uint16_t index, float value);
  static can_msgs::msg::Frame create_read_parameter_frame(uint8_t motor_id, uint16_t index);
  static can_msgs::msg::Frame create_enable_frame(uint8_t motor_id);
};

}  // namespace edulite05_driver
