#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "can_msgs/msg/frame.hpp"

namespace edulite05_driver {
constexpr uint8_t HOST_ID = 0xFD;
constexpr uint8_t TYPE_FEEDBACK = 0x02;
constexpr uint8_t TYPE_ENABLE = 0x03;
constexpr uint8_t TYPE_READ = 0x11;
constexpr uint8_t TYPE_WRITE = 0x12;
constexpr uint8_t RESET_STATUS_MODE = 0x00;
constexpr uint8_t RUN_STATUS_MODE = 0x02;
constexpr uint16_t RUN_MODE = 0x7005;
constexpr uint16_t SPEED_REFERENCE = 0x700A;
constexpr uint16_t POSITION_REFERENCE = 0x7016;
constexpr uint16_t SPEED_LIMIT = 0x7017;
constexpr uint16_t CURRENT_LIMIT = 0x7018;
constexpr uint16_t ACCELERATION = 0x7022;
constexpr uint16_t PP_SPEED = 0x7024;
constexpr uint16_t PP_ACCELERATION = 0x7025;
constexpr int MAX_INITIALIZATION_RETRIES = 10;
constexpr auto WRITE_SETTLING_TIME = std::chrono::milliseconds(2);
constexpr auto RESPONSE_TIMEOUT = std::chrono::milliseconds(100);
constexpr auto ERROR_RETRY_PERIOD = std::chrono::milliseconds(200);
constexpr float PI = 3.14159265358979323846f;

enum class ControlMode : uint8_t {
  PROFILE_POSITION = 1,
  VELOCITY = 2,
  CYCLIC_SYNCHRONOUS_POSITION = 5
};

enum class PositionReferenceMode : uint8_t { SERVICE, YAML_ABSOLUTE };

enum class MotorState : uint8_t { OFFLINE, INITIALIZING, READY, ERROR };

struct MotorConfig {
  std::string name;
  uint16_t logical_id = 0;
  uint8_t can_id = 0;
  ControlMode control_mode = ControlMode::VELOCITY;
  float current_limit = 11.0f;
  float acceleration = 150.0f;
  float speed_limit = 50.0f;
  uint32_t command_period_ms = 10;
  uint32_t target_timeout_ms = 200;
  uint32_t feedback_timeout_ms = 500;
  PositionReferenceMode position_reference_mode =
      PositionReferenceMode::SERVICE;
  float startup_absolute_position_rad = 0.0f;
  float minimum_position_rad = -1000.0f;
  float maximum_position_rad = 1000.0f;
};

struct MotorFeedback {
  float position = 0.0f;
  float velocity = 0.0f;
  float effort = 0.0f;
  float temperature = 0.0f;
  uint32_t fault_code = 0;
};

class Protocol {
public:
  explicit Protocol(const MotorConfig &config);

  const std::string &name() const { return config_.name; }
  uint16_t logical_id() const { return config_.logical_id; }
  uint8_t can_id() const { return config_.can_id; }
  ControlMode control_mode() const { return config_.control_mode; }
  MotorState state() const { return state_; }
  bool position_reference_is_set() const { return position_reference_is_set_; }
  bool is_connected() const { return connected_; }
  bool is_configured() const { return configured_; }
  bool is_enabled() const { return enabled_; }
  const MotorFeedback &feedback() const { return feedback_; }

  // VELOCITYでは速度[rad/s]、PP/CSPでは校正後の絶対位置[rad]を設定する。
  void set_target(float target);
  // 現在位置をcurrent_position_radとして扱うよう位置オフセットを更新する。
  bool set_current_position(float current_position_rad);

  std::optional<can_msgs::msg::Frame> create_initialization_frame();
  bool receive(const can_msgs::msg::Frame &message);
  std::optional<can_msgs::msg::Frame> create_target_frame();
  void watchdog();

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  enum class InitializationStep {
    WRITE_PARAMETER,
    WAIT_AFTER_WRITE,
    READ_PARAMETER,
    WAIT_FOR_READ,
    ENABLE,
    WAIT_FOR_ENABLE,
    READY,
    ERROR
  };

  struct InitializationParameter {
    uint16_t index;
    float value;
    bool is_uint8;
  };

  bool uses_position_control() const;
  void retry_initialization();
  void restart_initialization(bool clear_target);
  void process_feedback(const can_msgs::msg::Frame &message);
  void process_parameter_response(const can_msgs::msg::Frame &message);

  static can_msgs::msg::Frame make_base_frame(uint8_t type, uint8_t motor_id);
  static can_msgs::msg::Frame
  make_write_uint8_frame(uint8_t motor_id, uint16_t index, uint8_t value);
  static can_msgs::msg::Frame
  make_write_float_frame(uint8_t motor_id, uint16_t index, float value);
  static can_msgs::msg::Frame make_read_parameter_frame(uint8_t motor_id,
                                                        uint16_t index);
  static can_msgs::msg::Frame make_enable_frame(uint8_t motor_id);

  MotorConfig config_;
  MotorFeedback feedback_;
  MotorState state_ = MotorState::INITIALIZING;
  InitializationStep initialization_step_ = InitializationStep::WRITE_PARAMETER;
  std::vector<InitializationParameter> initialization_parameters_;
  std::size_t initialization_parameter_index_ = 0;
  float target_ = 0.0f;
  bool target_received_ = false;
  bool connected_ = false;
  bool configured_ = false;
  bool enabled_ = false;
  int initialization_retry_count_ = 0;
  float raw_position_ = 0.0f;
  float position_offset_ = 0.0f;
  bool position_reference_is_set_ = false;
  TimePoint last_feedback_time_{};
  TimePoint last_request_time_{};
  TimePoint last_target_time_{};
  TimePoint last_command_time_{};
  TimePoint error_time_{};
};
} // namespace edulite05_driver
