#include "edulite05_driver/edulite05_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace edulite05_driver {
namespace {
uint16_t read_big_endian_uint16(const std::array<uint8_t, 8> &data,
                                std::size_t index) {
  return (static_cast<uint16_t>(data[index]) << 8) |
         static_cast<uint16_t>(data[index + 1]);
}

float decode_uint16(uint16_t value, float minimum, float maximum) {
  return minimum + static_cast<float>(value) * (maximum - minimum) / 65535.0f;
}
}

Protocol::Protocol(const MotorConfig &config) : config_(config) {
  // 最初に必ずrun_mode
  initialization_parameters_.push_back(
      {RUN_MODE, static_cast<float>(static_cast<uint8_t>(config_.control_mode)),
       true});

  switch (config_.control_mode) {
  case ControlMode::VELOCITY:
  initialization_parameters_.push_back(
        {CURRENT_LIMIT, config_.current_limit, false});
  initialization_parameters_.push_back(
        {ACCELERATION, config_.acceleration, false});
    break;
  case ControlMode::CYCLIC_SYNCHRONOUS_POSITION:
  initialization_parameters_.push_back(
        {SPEED_LIMIT, config_.speed_limit, false});
  initialization_parameters_.push_back(
        {CURRENT_LIMIT, config_.current_limit, false});
    break;
  case ControlMode::PROFILE_POSITION:
  initialization_parameters_.push_back(
        {PP_SPEED, config_.speed_limit, false});
  initialization_parameters_.push_back(
        {PP_ACCELERATION, config_.acceleration, false});
  initialization_parameters_.push_back(
        {CURRENT_LIMIT, config_.current_limit, false});
    break;
  }
}

std::string Protocol::initialization_diagnostic() const {
  const char *step_name = "unknown";
  switch (initialization_step_) {
  case InitializationStep::WRITE_PARAMETER:
    step_name = "write";
    break;
  case InitializationStep::WAIT_AFTER_WRITE:
    step_name = "wait_after_write";
    break;
  case InitializationStep::READ_PARAMETER:
    step_name = "read";
    break;
  case InitializationStep::WAIT_FOR_READ:
    step_name = "wait_for_read";
    break;
  case InitializationStep::ENABLE:
    step_name = "enable";
    break;
  case InitializationStep::WAIT_FOR_ENABLE:
    step_name = "wait_for_enable";
    break;
  case InitializationStep::READY:
    step_name = "ready";
    break;
  case InitializationStep::ERROR:
    step_name = "error";
    break;
  }
  std::ostringstream stream;
  stream << "step=" << step_name;
  if (initialization_parameter_index_ < initialization_parameters_.size()) {
    stream << " register=0x" << std::hex << std::uppercase
           << initialization_parameters_[initialization_parameter_index_].index;
  }
  stream << std::dec << " retry=" << initialization_retry_count_;
  return stream.str();
}

void Protocol::set_target(float target) {
  if (!std::isfinite(target)) {
    return;
  }
  target_ = target;
  target_received_ = true;
  last_target_time_ = Clock::now();
}

bool Protocol::set_current_position(float current_position_rad) {
  if (!uses_position_control() || state_ != MotorState::READY ||
      !connected_ ||
      !std::isfinite(current_position_rad)) {
    return false;
  }
  const auto current_time = Clock::now();
  if (last_feedback_time_ == TimePoint{} ||
      current_time - last_feedback_time_ >
          std::chrono::milliseconds(config_.feedback_timeout_ms)) {
    return false;
  }

  // Only update the driver's coordinate conversion. Do not transmit anything
  // to the actuator until the upper controller supplies its next target.
  position_offset_ = current_position_rad - raw_position_;
  feedback_.position = current_position_rad;
  position_reference_is_set_ = true;
  provisional_position_reference_initialized_ = true;
  target_received_ = false;
  return true;
}

bool Protocol::uses_position_control() const {
  return config_.control_mode == ControlMode::PROFILE_POSITION ||
         config_.control_mode == ControlMode::CYCLIC_SYNCHRONOUS_POSITION;
}

std::optional<can_msgs::msg::Frame> Protocol::create_initialization_frame() {
  const auto current_time = Clock::now();

  switch (initialization_step_) {
  case InitializationStep::WRITE_PARAMETER: {
    state_ = MotorState::INITIALIZING;
    const auto &parameter = initialization_parameters_[initialization_parameter_index_];
    last_request_time_ = current_time;
    initialization_step_ = InitializationStep::WAIT_AFTER_WRITE;

    if (parameter.is_uint8) {
      return make_write_uint8_frame(config_.can_id, parameter.index,
                                    static_cast<uint8_t>(parameter.value));
    }
    return make_write_float_frame(config_.can_id, parameter.index,
                                  parameter.value);
  }
  case InitializationStep::WAIT_AFTER_WRITE:
    // これで帰ってきた応答そのものを設定成功判定には使わずに，type17 Readbackで確認
    if (current_time - last_request_time_ >= WRITE_SETTLING_TIME) {
      initialization_step_ = InitializationStep::READ_PARAMETER;
    }
    break;
  case InitializationStep::READ_PARAMETER: {
    const auto &parameter =
        initialization_parameters_[initialization_parameter_index_];
    last_request_time_ = current_time;
    initialization_step_ = InitializationStep::WAIT_FOR_READ;
    return make_read_parameter_frame(config_.can_id, parameter.index);
  }
  case InitializationStep::WAIT_FOR_READ:
    if (current_time - last_request_time_ > RESPONSE_TIMEOUT) {
      retry_initialization();
    }
    break;
  case InitializationStep::ENABLE:
    last_request_time_ = current_time;
    initialization_step_ = InitializationStep::WAIT_FOR_ENABLE;
    return make_enable_frame(config_.can_id);
  case InitializationStep::WAIT_FOR_ENABLE:
    if (current_time - last_request_time_ > RESPONSE_TIMEOUT) {
      retry_initialization();
    }
    break;
  case InitializationStep::READY:
    break;
  case InitializationStep::ERROR:
    // 電源再投入などでもROSノードを再起動しなくて済むように自動再試行
    if (current_time - error_time_ > ERROR_RETRY_PERIOD) {
      restart_initialization(true);
    }
    break;
  }
  return std::nullopt;
}

bool Protocol::receive(const can_msgs::msg::Frame &message) {
  if (!message.is_extended || message.dlc < 8) {
    return false;
  }
  const auto type = static_cast<uint8_t>((message.id >> 24) & 0x1F);
  const auto motor_id = static_cast<uint8_t>((message.id >> 8) & 0xFF);
  if (motor_id != config_.can_id) {
    return false;
  }
  if (type == TYPE_FEEDBACK) {
    process_feedback(message);
    return true;
  }
  if (type == TYPE_READ) {
    return process_parameter_response(message);
  }
  last_feedback_time_ = Clock::now();
  return false;
}

/// @brief フィードバックについてのデータを処理
/// @param message
void Protocol::process_feedback(const can_msgs::msg::Frame &message) {
  connected_ = true;
  last_feedback_time_ = Clock::now();
  const auto wrapped_position =
      decode_uint16(read_big_endian_uint16(message.data, 0), -4.0f * PI,
                    4.0f * PI);
  if (!raw_position_initialized_) {
    raw_position_ = wrapped_position;
    last_wrapped_position_ = wrapped_position;
    raw_position_initialized_ = true;
  } else {
    auto position_delta = wrapped_position - last_wrapped_position_;
    constexpr float feedback_position_period = 8.0f * PI;
    constexpr float feedback_position_half_period = 4.0f * PI;
    if (position_delta > feedback_position_half_period) {
      position_delta -= feedback_position_period;
    } else if (position_delta < -feedback_position_half_period) {
      position_delta += feedback_position_period;
    }
    raw_position_ += position_delta;
    last_wrapped_position_ = wrapped_position;
  }

  // PositionReferenceMode::YAML_OFFSETの場合は，初期化時に設定されたオフセットを使って絶対位置を計算する
  if (uses_position_control() &&
      !position_reference_is_set_ && config_.position_reference_mode == PositionReferenceMode::YAML_OFFSET) {
    position_offset_ = config_.position_offset_rad;
    position_reference_is_set_ = true;
  }

  // PositionReferenceMode::SERVICEの場合は，初期化時に設定されたオフセットを使って絶対位置を計算する
  if (uses_position_control() &&
      !position_reference_is_set_ && config_.allow_unreferenced_position_commands &&
      !provisional_position_reference_initialized_) {
    position_offset_ = -raw_position_;
    provisional_position_reference_initialized_ = true;
  }

  feedback_.position = raw_position_ + position_offset_;
  feedback_.velocity =
      decode_uint16(read_big_endian_uint16(message.data, 2), -50.0f, 50.0f);
  feedback_.torque_nm =
      decode_uint16(read_big_endian_uint16(message.data, 4), -6.0f, 6.0f);
  feedback_.temperature =
      static_cast<float>(read_big_endian_uint16(message.data, 6)) / 10.0f;

  // Type2 ID bit21~16
  feedback_.fault_code = static_cast<uint32_t>((message.id >> 16) & 0x3F);
  // bit23~22
  const auto mode_status = static_cast<uint8_t>((message.id >> 22) & 0x03);

  // Enable完了確認

  if (initialization_step_ == InitializationStep::WAIT_FOR_ENABLE) {
    if (mode_status == RUN_STATUS_MODE) {
      enabled_ = true;
      initialization_retry_count_ = 0;
      if (initialization_parameter_index_ >=
          initialization_parameters_.size()) {
        state_ = MotorState::READY;
        initialization_step_ = InitializationStep::READY;
      } else {
        initialization_step_ = InitializationStep::WRITE_PARAMETER;
      }
    }
    return;
  }

  // PP/CSPは新しい位置指令が届くまで非RUNを返すため、RUN statusを
  // 再接続判定に使用しない。位置制御モーターの電源再投入はwatchdogの
  // feedback timeoutで検出する。
  if (initialization_step_ == InitializationStep::READY) {
    if (uses_position_control()) {
      return;
    }
    if (mode_status == RUN_STATUS_MODE) {
      consecutive_non_run_feedback_count_ = 0;
    } else {
      ++consecutive_non_run_feedback_count_;
      // PPモードはREADY直後に一時的な非RUN状態を返すことがある。
      // 上位ノードが最初の位置指令を送る前に再初期化しないようデバウンスする。
      constexpr int non_run_feedback_restart_threshold = 20;
      if (consecutive_non_run_feedback_count_ >=
          non_run_feedback_restart_threshold) {
        restart_initialization(true);
      }
    }
  }
}

bool Protocol::process_parameter_response(const can_msgs::msg::Frame &message) {
  connected_ = true;
  last_feedback_time_ = Clock::now();
  const auto destination = static_cast<uint8_t>(message.id & 0xFF);
  const auto status = static_cast<uint8_t>((message.id >> 16) & 0xFF);
  if (destination != HOST_ID) {
    return false;
  }
  const auto index = static_cast<uint16_t>(message.data[0]) |
                     (static_cast<uint16_t>(message.data[1]) << 8);
  if (initialization_step_ == InitializationStep::READY &&
      config_.current_feedback_enabled && index == CURRENT_FEEDBACK &&
      status == RESET_STATUS_MODE) {
    float value = 0.0f;
    std::memcpy(&value, message.data.data() + 4, sizeof(float));
    if (std::isfinite(value)) {
      feedback_.current_a = value;
      return true;
    }
    return false;
  }
  if (initialization_step_ != InitializationStep::WAIT_FOR_READ) {
    return false;
  }
  // Type17 status != 0
  if (status != RESET_STATUS_MODE) {
    retry_initialization();
    return false;
  }

  const auto &expected =
      initialization_parameters_[initialization_parameter_index_];
  // 古い別parameterの応答などは無視
  if (index != expected.index) {
    return false;
  }

  bool values_match = false;
  if (expected.is_uint8) {
    values_match = message.data[4] == static_cast<uint8_t>(expected.value);
  } else {
    float value = 0.0f;
    std::memcpy(&value, message.data.data() + 4, sizeof(float));
    values_match = std::fabs(value - expected.value) < 0.001f;
  }
  if (!values_match) {
    retry_initialization();
    return false;
  }

  initialization_retry_count_ = 0;
  ++initialization_parameter_index_;
  if (initialization_parameter_index_ >= initialization_parameters_.size()) {
    if (enabled_) {
      state_ = MotorState::READY;
      initialization_step_ = InitializationStep::READY;
    } else {
      initialization_step_ = InitializationStep::ENABLE;
    }
  } else if (!enabled_) {
    initialization_step_ = InitializationStep::ENABLE;
  } else {
    initialization_step_ = InitializationStep::WRITE_PARAMETER;
  }
  // 初期化状態が進んだことをNodeへ通知し、特に最終readbackで
  // READYになったstateを個別topicへ即時publishさせる。
  return true;
}

std::optional<can_msgs::msg::Frame> Protocol::create_current_feedback_frame() {
  if (!config_.current_feedback_enabled || state_ != MotorState::READY) {
    return std::nullopt;
  }
  const auto current_time = Clock::now();
  const auto period =
      std::chrono::milliseconds(config_.current_feedback_period_ms);
  if (last_current_feedback_request_time_ != TimePoint{} &&
      current_time - last_current_feedback_request_time_ < period) {
    return std::nullopt;
  }
  last_current_feedback_request_time_ = current_time;
  return make_read_parameter_frame(config_.can_id, CURRENT_FEEDBACK);
}

void Protocol::retry_initialization() {
  ++initialization_retry_count_;
  if (initialization_retry_count_ > MAX_INITIALIZATION_RETRIES) {
    state_ = MotorState::ERROR;
    enabled_ = false;
    initialization_step_ = InitializationStep::ERROR;
    error_time_ = Clock::now();
    return;
  }
  if (initialization_step_ == InitializationStep::WAIT_FOR_ENABLE) {
    initialization_step_ = InitializationStep::ENABLE;
  } else {
    // Writeからやり直す
    initialization_step_ = InitializationStep::WRITE_PARAMETER;
  }
}

void Protocol::restart_initialization(bool clear_target) {
  state_ = MotorState::INITIALIZING;
  enabled_ = false;
  consecutive_non_run_feedback_count_ = 0;
  initialization_parameter_index_ = 0;
  initialization_retry_count_ = 0;
  initialization_step_ = InitializationStep::WRITE_PARAMETER;
  if (uses_position_control()) {
    position_reference_is_set_ = false;
    provisional_position_reference_initialized_ = false;
    raw_position_ = 0.0f;
    last_wrapped_position_ = 0.0f;
    raw_position_initialized_ = false;
  }
  if (clear_target) {
    target_received_ = false;
    target_ = 0.0f;
  }
}

std::optional<can_msgs::msg::Frame> Protocol::create_target_frame() {
  if (state_ != MotorState::READY || !target_received_) {
    return std::nullopt;
  }

  const auto current_time = Clock::now();
  const auto command_period = std::chrono::milliseconds(config_.command_period_ms);
  if (last_command_time_ != TimePoint{} && current_time - last_command_time_ < command_period) {
    return std::nullopt;
  }
  if (uses_position_control() && !position_reference_is_set_ &&
      !(config_.allow_unreferenced_position_commands &&
        provisional_position_reference_initialized_)) {
    return std::nullopt;
  }

  last_command_time_ = current_time;

  if (config_.control_mode == ControlMode::VELOCITY) {
    auto velocity_target = target_;
    // 上位制御ノードが死んだ場合は停止
    if (current_time - last_target_time_ > std::chrono::milliseconds(config_.target_timeout_ms)) {
      velocity_target = 0.0f;
    }
    velocity_target = std::clamp(velocity_target, -config_.speed_limit, config_.speed_limit);
    return make_write_float_frame(config_.can_id, SPEED_REFERENCE, velocity_target);
  }

  // PP / CSPは指令が途絶えても最後の位置を保持する
  const auto absolute_position_target = std::clamp(target_, config_.minimum_position_rad, config_.maximum_position_rad);
  const auto motor_position_target = absolute_position_target - position_offset_;
  return make_write_float_frame(config_.can_id, POSITION_REFERENCE,  motor_position_target);
}

void Protocol::watchdog() {
  if (state_ != MotorState::READY || !connected_) {
    return;
  }
  if (Clock::now() - last_feedback_time_ <= std::chrono::milliseconds(config_.feedback_timeout_ms)) {
    return;
  }
  connected_ = false;
  // 再接続後に古いtargetで突然動かないように
  // targetも破棄
  restart_initialization(true);
}

can_msgs::msg::Frame Protocol::make_base_frame(uint8_t type, uint8_t motor_id) {
  can_msgs::msg::Frame message;
  message.id = (static_cast<uint32_t>(type) << 24) |
               (static_cast<uint32_t>(HOST_ID) << 8) | motor_id;
  message.is_extended = true;
  message.dlc = 8;
  message.data.fill(0);
  return message;
}

can_msgs::msg::Frame Protocol::make_write_uint8_frame(uint8_t motor_id,
                                                      uint16_t index,
                                                      uint8_t value) {
  auto message = make_base_frame(TYPE_WRITE, motor_id);
  message.data[0] = static_cast<uint8_t>(index & 0xFF);
  message.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  message.data[4] = value;
  return message;
}

can_msgs::msg::Frame Protocol::make_write_float_frame(uint8_t motor_id,
                                                      uint16_t index,
                                                      float value) {
  auto message = make_base_frame(TYPE_WRITE, motor_id);
  message.data[0] = static_cast<uint8_t>(index & 0xFF);
  message.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  std::memcpy(message.data.data() + 4, &value, sizeof(float));
  return message;
}

can_msgs::msg::Frame Protocol::make_read_parameter_frame(uint8_t motor_id,
                                                         uint16_t index) {
  auto message = make_base_frame(TYPE_READ, motor_id);
  message.data[0] = static_cast<uint8_t>(index & 0xFF);
  message.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  return message;
}

can_msgs::msg::Frame Protocol::make_enable_frame(uint8_t motor_id) {
  return make_base_frame(TYPE_ENABLE, motor_id);
}
}  // namespace edulite05_driver
