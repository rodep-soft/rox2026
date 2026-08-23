#include "edulite05_driver/edulite05_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace edulite05_driver
{
namespace
{
uint16_t read_big_endian_uint16(
  const std::array<uint8_t, 8> & data,
  std::size_t index)
{
  return (static_cast<uint16_t>(data[index]) << 8) |
         static_cast<uint16_t>(data[index + 1]);
}

uint32_t read_little_endian_uint32(
  const std::array<uint8_t, 8> & data,
  std::size_t index)
{
  return static_cast<uint32_t>(data[index]) |
         (static_cast<uint32_t>(data[index + 1]) << 8) |
         (static_cast<uint32_t>(data[index + 2]) << 16) |
         (static_cast<uint32_t>(data[index + 3]) << 24);
}

float decode_uint16(uint16_t value, float minimum, float maximum)
{
  return minimum + static_cast<float>(value) * (maximum - minimum) / 65535.0f;
}
} // namespace

Protocol::Protocol(const MotorConfig & config)
: config_(config)
{
  if (uses_position_control()) {
    position_reference_state_ = PositionReferenceState::UNAVAILABLE;
  }
  // 最初に必ずrun_mode
  initialization_parameters_.push_back(
    {RUN_MODE, InitializationParameterType::UINT8,
      static_cast<float>(static_cast<uint8_t>(config_.control_mode))});

  switch (config_.control_mode) {
    case ControlMode::VELOCITY:
      initialization_parameters_.push_back(
        {SPEED_REFERENCE, InitializationParameterType::FLOAT, 0.0f});
      initialization_parameters_.push_back(
        {CURRENT_LIMIT, InitializationParameterType::FLOAT, config_.current_limit});
      initialization_parameters_.push_back(
        {ACCELERATION, InitializationParameterType::FLOAT, config_.acceleration});
      break;
    case ControlMode::CYCLIC_SYNCHRONOUS_POSITION:
      initialization_parameters_.push_back(
        {SPEED_LIMIT, InitializationParameterType::FLOAT, config_.speed_limit});
      initialization_parameters_.push_back(
        {CURRENT_LIMIT, InitializationParameterType::FLOAT, config_.current_limit});
      break;
    case ControlMode::PROFILE_POSITION:
      initialization_parameters_.push_back(
        {SPEED_LIMIT, InitializationParameterType::FLOAT, config_.speed_limit});
      initialization_parameters_.push_back(
        {CURRENT_LIMIT, InitializationParameterType::FLOAT, config_.current_limit});
      break;
  }
}

std::string Protocol::initialization_diagnostic() const
{
  const char * step_name = "unknown";
  switch (initialization_step_) {
    case InitializationStep::RESET_MOTOR:
      step_name = "reset_motor";
      break;
    case InitializationStep::WAIT_AFTER_RESET:
      step_name = "wait_after_reset";
      break;
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
    case InitializationStep::READ_STARTUP_POSITION:
      step_name = "read_startup_position";
      break;
    case InitializationStep::WAIT_FOR_STARTUP_POSITION:
      step_name = "wait_for_startup_position";
      break;
    case InitializationStep::WRITE_STARTUP_HOLD:
      step_name = "write_startup_hold";
      break;
    case InitializationStep::WAIT_AFTER_STARTUP_HOLD_WRITE:
      step_name = "wait_after_startup_hold_write";
      break;
    case InitializationStep::READ_STARTUP_HOLD:
      step_name = "read_startup_hold";
      break;
    case InitializationStep::WAIT_FOR_STARTUP_HOLD:
      step_name = "wait_for_startup_hold";
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
  if (initialization_step_ == InitializationStep::READ_STARTUP_POSITION ||
    initialization_step_ == InitializationStep::WAIT_FOR_STARTUP_POSITION)
  {
    stream << " register=0x" << std::hex << std::uppercase
           << MECHANICAL_POSITION;
  } else if (initialization_step_ == InitializationStep::WRITE_STARTUP_HOLD ||
    initialization_step_ == InitializationStep::WAIT_AFTER_STARTUP_HOLD_WRITE ||
    initialization_step_ == InitializationStep::READ_STARTUP_HOLD ||
    initialization_step_ == InitializationStep::WAIT_FOR_STARTUP_HOLD)
  {
    stream << " register=0x" << std::hex << std::uppercase
           << POSITION_REFERENCE;
  } else if (initialization_parameter_index_ <
    initialization_parameters_.size())
  {
    stream << " register=0x" << std::hex << std::uppercase
           << initialization_parameters_[initialization_parameter_index_].index;
  }
  stream << std::dec << " retry=" << initialization_retry_count_;
  return stream.str();
}

void Protocol::set_target(float target)
{
  if (!std::isfinite(target)) {
    return;
  }
  // A position target received during initialization could be applied immediately when the
  // motor becomes READY, before the controller observes the new measured position. Discard it
  // so a power-cycle cannot revive a stale absolute target and cause a position jump.
  if (uses_position_control() && state_ != MotorState::READY) {
    return;
  }
  target_value_ = target;
  has_target_ = true;
  last_target_time_ = Clock::now();
}

bool Protocol::set_current_position(float current_position_rad)
{
  if (!uses_position_control() || state_ != MotorState::READY ||
    !feedback_connected_ || !std::isfinite(current_position_rad))
  {
    return false;
  }
  const auto current_time = Clock::now();
  if (last_feedback_time_ == TimePoint{} ||
    current_time - last_feedback_time_ >
    std::chrono::milliseconds(config_.feedback_timeout_ms))
  {
    return false;
  }

  logical_position_offset_rad_ = current_position_rad - motor_position_rad_;
  feedback_.position = current_position_rad;
  position_reference_state_ = PositionReferenceState::ESTABLISHED;
  has_target_ = false;
  return true;
}

bool Protocol::uses_position_control() const
{
  return config_.control_mode == ControlMode::PROFILE_POSITION ||
         config_.control_mode == ControlMode::CYCLIC_SYNCHRONOUS_POSITION;
}

bool Protocol::position_command_is_allowed() const
{
  return !uses_position_control() ||
         position_reference_state_ == PositionReferenceState::TEMPORARY ||
         position_reference_state_ == PositionReferenceState::ESTABLISHED;
}

std::optional<can_msgs::msg::Frame> Protocol::create_initialization_frame(
  TimePoint current_time)
{
  // 待機完了やタイムアウト処理だけで呼び出しを終えずに送信が必要な状態まで同じ呼び出し内で進め，送信フレームは必ず最大1つ
  while (true) {
    switch (initialization_step_) {
      case InitializationStep::RESET_MOTOR:
        state_ = MotorState::INITIALIZING;
        last_request_time_ = current_time;
        initialization_step_ = InitializationStep::WAIT_AFTER_RESET;
        return make_reset_frame(config_.can_id);

      case InitializationStep::WAIT_AFTER_RESET:
        if (current_time - last_request_time_ < std::chrono::milliseconds(10)) {
          return std::nullopt;
        }
        initialization_step_ = InitializationStep::WRITE_PARAMETER;
        continue;

      case InitializationStep::WRITE_PARAMETER: {
          state_ = MotorState::INITIALIZING;
          const auto & parameter =
            initialization_parameters_[initialization_parameter_index_];
          last_request_time_ = current_time;
          initialization_step_ = InitializationStep::WAIT_AFTER_WRITE;

          if (parameter.type == InitializationParameterType::UINT8) {
            return make_write_uint8_frame(
              config_.can_id, parameter.index,
              static_cast<uint8_t>(parameter.value));
          }
          return make_write_float_frame(
            config_.can_id, parameter.index, parameter.value);
        }

      case InitializationStep::WAIT_AFTER_WRITE:
        // 書込み応答そのものではなく，settling後のType 17 Readbackで確認
        if (current_time - last_request_time_ < WRITE_SETTLING_TIME) {
          return std::nullopt;
        }
        initialization_step_ = InitializationStep::READ_PARAMETER;
        continue;

      case InitializationStep::READ_PARAMETER: {
          const auto & parameter =
            initialization_parameters_[initialization_parameter_index_];
          last_request_time_ = current_time;
          initialization_step_ = InitializationStep::WAIT_FOR_READ;
          return make_read_parameter_frame(config_.can_id, parameter.index);
        }

      case InitializationStep::WAIT_FOR_READ:
        if (current_time - last_request_time_ <= RESPONSE_TIMEOUT) {
          return std::nullopt;
        }
        retry_initialization();
        continue;

      case InitializationStep::READ_STARTUP_POSITION:
        last_request_time_ = current_time;
        initialization_step_ =
          InitializationStep::WAIT_FOR_STARTUP_POSITION;
        return make_read_parameter_frame(config_.can_id, MECHANICAL_POSITION);

      case InitializationStep::WAIT_FOR_STARTUP_POSITION:
        if (current_time - last_request_time_ <= RESPONSE_TIMEOUT) {
          return std::nullopt;
        }
        retry_initialization();
        continue;

      case InitializationStep::WRITE_STARTUP_HOLD:
        last_request_time_ = current_time;
        initialization_step_ =
          InitializationStep::WAIT_AFTER_STARTUP_HOLD_WRITE;
        return make_write_float_frame(
          config_.can_id, POSITION_REFERENCE, startup_hold_position_rad_);

      case InitializationStep::WAIT_AFTER_STARTUP_HOLD_WRITE:
        if (current_time - last_request_time_ < WRITE_SETTLING_TIME) {
          return std::nullopt;
        }
        initialization_step_ = InitializationStep::READ_STARTUP_HOLD;
        continue;

      case InitializationStep::READ_STARTUP_HOLD:
        last_request_time_ = current_time;
        initialization_step_ = InitializationStep::WAIT_FOR_STARTUP_HOLD;
        return make_read_parameter_frame(config_.can_id, POSITION_REFERENCE);

      case InitializationStep::WAIT_FOR_STARTUP_HOLD:
        if (current_time - last_request_time_ <= RESPONSE_TIMEOUT) {
          return std::nullopt;
        }
        retry_initialization();
        continue;

      case InitializationStep::ENABLE:
        last_request_time_ = current_time;
        initialization_step_ = InitializationStep::WAIT_FOR_ENABLE;
        return make_enable_frame(config_.can_id);

      case InitializationStep::WAIT_FOR_ENABLE:
        if (current_time - last_request_time_ <= RESPONSE_TIMEOUT) {
          return std::nullopt;
        }
        retry_initialization();
        continue;

      case InitializationStep::READY:
        return std::nullopt;

      case InitializationStep::ERROR:
        // fault継続中はprocess_feedback()がerror_time_を更新
        if (current_time - error_time_ <= ERROR_RETRY_PERIOD) {
          return std::nullopt;
        }
        restart_initialization(true);
        continue;
    }
  }
}

bool Protocol::receive(const can_msgs::msg::Frame & message)
{
  if (!message.is_extended || message.is_rtr || message.is_error ||
    message.dlc != 8)
  {
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
  if (type == TYPE_FAULT) {
    return process_fault(message);
  }
  return false;
}

/// @brief フィードバックについてのデータを処理
/// @param message
void Protocol::process_feedback(const can_msgs::msg::Frame & message)
{
  const auto current_time = Clock::now();
  feedback_connected_ = true;
  last_feedback_time_ = current_time;
  const auto wrapped_position = decode_uint16(
    read_big_endian_uint16(message.data, 0), -4.0f * PI, 4.0f * PI);
  if (!motor_position_initialized_) {
    motor_position_rad_ = startup_position_alignment_pending_ ?
      startup_hold_position_rad_ : wrapped_position;
    startup_position_alignment_pending_ = false;
    last_wrapped_position_rad_ = wrapped_position;
    motor_position_initialized_ = true;
  } else {
    auto position_delta = wrapped_position - last_wrapped_position_rad_;
    constexpr float feedback_position_period = 8.0f * PI;
    constexpr float feedback_position_half_period = 4.0f * PI;
    if (position_delta > feedback_position_half_period) {
      position_delta -= feedback_position_period;
    } else if (position_delta < -feedback_position_half_period) {
      position_delta += feedback_position_period;
    }
    motor_position_rad_ += position_delta;
    last_wrapped_position_rad_ = wrapped_position;
  }

  if (uses_position_control() &&
    position_reference_state_ == PositionReferenceState::UNAVAILABLE)
  {
    if (config_.position_reference_source ==
      PositionReferenceSource::YAML_OFFSET)
    {
      // YAMLの固定オフセットは，そのまま正式な位置基準として使用
      logical_position_offset_rad_ = config_.position_offset_rad;
      position_reference_state_ = PositionReferenceState::ESTABLISHED;
    } else {
      // serviceで原点を確定するまでは，最初の位置を一時原点として
      // ホーミング用の位置指令だけを許可
      logical_position_offset_rad_ = -motor_position_rad_;
      position_reference_state_ = PositionReferenceState::TEMPORARY;
    }
  }

  feedback_.position = motor_position_rad_ + logical_position_offset_rad_;
  feedback_.velocity =
    decode_uint16(read_big_endian_uint16(message.data, 2), -50.0f, 50.0f);
  feedback_.torque_nm =
    decode_uint16(read_big_endian_uint16(message.data, 4), -6.0f, 6.0f);
  feedback_.temperature =
    static_cast<float>(read_big_endian_uint16(message.data, 6)) / 10.0f;

  // Type 2は6bitの故障要約のため，Type 21の詳細値があれば継続して保持
  const auto summary_fault_code =
    static_cast<uint32_t>((message.id >> 16) & 0x3F);
  // bit23~22
  const auto mode_status = static_cast<uint8_t>((message.id >> 22) & 0x03);

  if (!startup_run_state_observed_) {
    motor_was_running_at_startup_ = mode_status == RUN_STATUS_MODE;
    startup_run_state_observed_ = true;
  }

  if (summary_fault_code != 0U) {
    const auto fault_code = detailed_fault_code_ != 0U ?
      detailed_fault_code_ : summary_fault_code;
    enter_fault_state(fault_code, current_time);
    return;
  }
  detailed_fault_code_ = 0;
  feedback_.fault_code = 0;

  // Enable完了確認

  if (initialization_step_ == InitializationStep::WAIT_FOR_ENABLE) {
    if (mode_status == RUN_STATUS_MODE) {
      motor_enabled_ = true;
      initialization_retry_count_ = 0;
      if (initialization_parameter_index_ >=
        initialization_parameters_.size())
      {
        state_ = MotorState::READY;
        initialization_step_ = InitializationStep::READY;
      } else {
        initialization_step_ = InitializationStep::WRITE_PARAMETER;
      }
    }
    return;
  }

  // PP/CSPは新しい位置指令が届くまで非RUNを返すため，RUN statusを再接続判定に使用しない
  // 位置制御モーターの電源再投入はwatchdogのfeedback timeoutで検出
  if (initialization_step_ == InitializationStep::READY) {
    if (uses_position_control()) {
      return;
    }
    if (mode_status == RUN_STATUS_MODE) {
      consecutive_non_run_feedback_count_ = 0;
    } else {
      ++consecutive_non_run_feedback_count_;
      // PPモードはREADY直後に一時的な非RUN状態を返すことがある
      // 上位ノードが最初の位置指令を送る前に再初期化しないようデバウンス
      constexpr int non_run_feedback_restart_threshold = 20;
      if (consecutive_non_run_feedback_count_ >=
        non_run_feedback_restart_threshold)
      {
        restart_initialization(true);
      }
    }
  }
}

bool Protocol::process_parameter_response(const can_msgs::msg::Frame & message)
{
  const auto destination = static_cast<uint8_t>(message.id & 0xFF);
  const auto fault_code = static_cast<uint8_t>((message.id >> 16) & 0x3F);
  if (destination != HOST_ID) {
    return false;
  }

  feedback_connected_ = true;
  last_feedback_time_ = Clock::now();
  const auto index = static_cast<uint16_t>(message.data[0]) |
    (static_cast<uint16_t>(message.data[1]) << 8);
  if (initialization_step_ == InitializationStep::READY &&
    config_.current_feedback_enabled && index == CURRENT_FEEDBACK &&
    fault_code == 0)
  {
    float value = 0.0f;
    std::memcpy(&value, message.data.data() + 4, sizeof(float));
    if (std::isfinite(value)) {
      feedback_.current_a = value;
      last_current_feedback_time_ = Clock::now();
      return true;
    }
    return false;
  }
  if (initialization_step_ == InitializationStep::WAIT_FOR_STARTUP_POSITION) {
    if (index != MECHANICAL_POSITION) {
      return false;
    }
    float current_position = 0.0f;
    std::memcpy(&current_position, message.data.data() + 4, sizeof(float));
    if (fault_code != 0 || !std::isfinite(current_position)) {
      retry_initialization();
      return false;
    }

    startup_hold_position_rad_ = current_position;
    if (motor_position_initialized_) {
      // MECHANICAL_POSITION is multi-turn. Align the accumulated feedback position with it;
      // otherwise the temporary zero can differ by whole 8*PI turns after a driver restart.
      motor_position_rad_ = current_position;
      if (uses_position_control() &&
        config_.position_reference_source == PositionReferenceSource::SET_POSITION_SERVICE &&
        position_reference_state_ != PositionReferenceState::ESTABLISHED)
      {
        logical_position_offset_rad_ = -motor_position_rad_;
        feedback_.position = 0.0f;
        // If the motor was already RUN before this driver initialized, only the software was
        // restarted. Preserve motion continuity by accepting the measured position as the new
        // logical reference. A real motor power-cycle is not RUN here and still requires homing.
        if (motor_was_running_at_startup_) {
          position_reference_state_ = PositionReferenceState::ESTABLISHED;
        }
      }
    } else {
      startup_position_alignment_pending_ = true;
    }
    initialization_step_ = InitializationStep::WRITE_STARTUP_HOLD;
    return true;
  }

  if (initialization_step_ == InitializationStep::WAIT_FOR_STARTUP_HOLD) {
    if (index != POSITION_REFERENCE) {
      return false;
    }
    float hold_position = 0.0f;
    std::memcpy(&hold_position, message.data.data() + 4, sizeof(float));
    if (fault_code != 0 || !std::isfinite(hold_position) ||
      std::fabs(hold_position - startup_hold_position_rad_) >= 0.001f)
    {
      retry_initialization();
      return false;
    }

    initialization_retry_count_ = 0;
    initialization_step_ = InitializationStep::ENABLE;
    return true;
  }

  if (initialization_step_ != InitializationStep::WAIT_FOR_READ) {
    return false;
  }
  // Type17 fault summary != 0
  if (fault_code != 0) {
    retry_initialization();
    return false;
  }

  const auto & expected =
    initialization_parameters_[initialization_parameter_index_];
  // 古い別parameterの応答などは無視
  if (index != expected.index) {
    return false;
  }

  bool values_match = false;
  if (expected.type == InitializationParameterType::UINT8) {
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
    if (motor_enabled_) {
      state_ = MotorState::READY;
      initialization_step_ = InitializationStep::READY;
    } else {
      initialization_step_ = InitializationStep::ENABLE;
    }
  } else if (!motor_enabled_) {
    if (uses_position_control() && expected.index == RUN_MODE) {
      initialization_step_ = InitializationStep::READ_STARTUP_POSITION;
    } else {
      initialization_step_ = InitializationStep::ENABLE;
    }
  } else {
    initialization_step_ = InitializationStep::WRITE_PARAMETER;
  }
  // 初期化状態が進んだことをNodeへ通知し，特に最終readbackでREADYになったstateを個別topicへ即時publish
  return true;
}

bool Protocol::process_fault(const can_msgs::msg::Frame & message)
{
  const auto fault_code = read_little_endian_uint32(message.data, 0);
  if (fault_code == 0U) {
    return false;
  }

  const auto current_time = Clock::now();
  feedback_connected_ = true;
  last_feedback_time_ = current_time;
  detailed_fault_code_ = fault_code;
  enter_fault_state(fault_code, current_time);
  return true;
}

void Protocol::enter_fault_state(uint32_t fault_code, TimePoint current_time)
{
  feedback_.fault_code = fault_code;
  feedback_.current_a = std::numeric_limits<float>::quiet_NaN();
  last_current_feedback_time_ = TimePoint{};
  if (state_ != MotorState::ERROR) {
    state_ = MotorState::ERROR;
    error_time_ = current_time;
  }
  motor_enabled_ = false;
  has_target_ = false;
  target_value_ = 0.0f;
  initialization_step_ = InitializationStep::ERROR;
}

void Protocol::invalidate_stale_current(TimePoint current_time)
{
  if (!config_.current_feedback_enabled ||
    last_current_feedback_time_ == TimePoint{})
  {
    return;
  }
  const auto timeout =
    std::chrono::milliseconds(config_.current_feedback_period_ms) *
    CURRENT_FEEDBACK_TIMEOUT_PERIODS;
  if (current_time - last_current_feedback_time_ <= timeout) {
    return;
  }

  feedback_.current_a = std::numeric_limits<float>::quiet_NaN();
  last_current_feedback_time_ = TimePoint{};
}

std::optional<can_msgs::msg::Frame> Protocol::create_current_feedback_frame(
  TimePoint current_time)
{
  if (!config_.current_feedback_enabled || state_ != MotorState::READY) {
    return std::nullopt;
  }
  const auto period =
    std::chrono::milliseconds(config_.current_feedback_period_ms);
  if (last_current_feedback_request_time_ != TimePoint{} &&
    current_time - last_current_feedback_request_time_ < period)
  {
    return std::nullopt;
  }
  last_current_feedback_request_time_ = current_time;
  return make_read_parameter_frame(config_.can_id, CURRENT_FEEDBACK);
}

void Protocol::retry_initialization()
{
  ++initialization_retry_count_;
  if (initialization_retry_count_ > MAX_INITIALIZATION_RETRIES) {
    state_ = MotorState::ERROR;
    motor_enabled_ = false;
    initialization_step_ = InitializationStep::ERROR;
    error_time_ = Clock::now();
    return;
  }
  if (initialization_step_ == InitializationStep::WAIT_FOR_ENABLE) {
    initialization_step_ = InitializationStep::ENABLE;
  } else if (initialization_step_ ==
    InitializationStep::WAIT_FOR_STARTUP_POSITION ||
    initialization_step_ == InitializationStep::WAIT_FOR_STARTUP_HOLD)
  {
    initialization_step_ = InitializationStep::READ_STARTUP_POSITION;
  } else {
    // Writeからやり直す
    initialization_step_ = InitializationStep::WRITE_PARAMETER;
  }
}

void Protocol::restart_initialization(bool clear_target)
{
  state_ = MotorState::INITIALIZING;
  motor_enabled_ = false;
  startup_run_state_observed_ = false;
  motor_was_running_at_startup_ = false;
  consecutive_non_run_feedback_count_ = 0;
  initialization_parameter_index_ = 0;
  initialization_retry_count_ = 0;
  detailed_fault_code_ = 0;
  initialization_step_ = InitializationStep::RESET_MOTOR;
  feedback_.current_a = std::numeric_limits<float>::quiet_NaN();
  last_current_feedback_time_ = TimePoint{};
  if (uses_position_control()) {
    position_reference_state_ = PositionReferenceState::UNAVAILABLE;
    motor_position_rad_ = 0.0f;
    last_wrapped_position_rad_ = 0.0f;
    motor_position_initialized_ = false;
    startup_position_alignment_pending_ = false;
  }
  if (clear_target) {
    has_target_ = false;
    target_value_ = 0.0f;
  }
}

std::optional<can_msgs::msg::Frame> Protocol::create_target_frame(
  TimePoint current_time)
{
  if (state_ != MotorState::READY || !has_target_) {
    return std::nullopt;
  }
  const auto command_period =
    std::chrono::milliseconds(config_.command_period_ms);
  if (last_command_time_ != TimePoint{} &&
    current_time - last_command_time_ < command_period)
  {
    return std::nullopt;
  }
  if (!position_command_is_allowed()) {
    return std::nullopt;
  }

  last_command_time_ = current_time;

  if (config_.control_mode == ControlMode::VELOCITY) {
    auto velocity_target = target_value_;
    // 上位制御ノードが死んだ場合は停止
    if (current_time - last_target_time_ >
      std::chrono::milliseconds(config_.target_timeout_ms))
    {
      velocity_target = 0.0f;
    }
    velocity_target =
      std::clamp(velocity_target, -config_.speed_limit, config_.speed_limit);
    return make_write_float_frame(
      config_.can_id, SPEED_REFERENCE,
      velocity_target);
  }

  // PP / CSPは指令が途絶えても最後の位置を保持
  const auto absolute_position_target =
    std::clamp(
    target_value_, config_.minimum_position_rad,
    config_.maximum_position_rad);
  const auto motor_position_target =
    absolute_position_target - logical_position_offset_rad_;
  return make_write_float_frame(
    config_.can_id, POSITION_REFERENCE,
    motor_position_target);
}

void Protocol::watchdog(TimePoint current_time)
{
  invalidate_stale_current(current_time);

  if (state_ != MotorState::READY || !feedback_connected_) {
    return;
  }
  if (current_time - last_feedback_time_ <=
    std::chrono::milliseconds(config_.feedback_timeout_ms))
  {
    return;
  }
  feedback_connected_ = false;
  // 再接続後に古いtargetで突然動かないようにtargetも破棄
  restart_initialization(true);
}

can_msgs::msg::Frame Protocol::make_base_frame(uint8_t type, uint8_t motor_id)
{
  can_msgs::msg::Frame message;
  message.id = (static_cast<uint32_t>(type) << 24) |
    (static_cast<uint32_t>(HOST_ID) << 8) | motor_id;
  message.is_extended = true;
  message.dlc = 8;
  message.data.fill(0);
  return message;
}

can_msgs::msg::Frame Protocol::make_write_uint8_frame(
  uint8_t motor_id,
  uint16_t index,
  uint8_t value)
{
  auto message = make_base_frame(TYPE_WRITE, motor_id);
  message.data[0] = static_cast<uint8_t>(index & 0xFF);
  message.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  message.data[4] = value;
  return message;
}

can_msgs::msg::Frame Protocol::make_write_float_frame(
  uint8_t motor_id,
  uint16_t index,
  float value)
{
  auto message = make_base_frame(TYPE_WRITE, motor_id);
  message.data[0] = static_cast<uint8_t>(index & 0xFF);
  message.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  std::memcpy(message.data.data() + 4, &value, sizeof(float));
  return message;
}

can_msgs::msg::Frame Protocol::make_read_parameter_frame(
  uint8_t motor_id,
  uint16_t index)
{
  auto message = make_base_frame(TYPE_READ, motor_id);
  message.data[0] = static_cast<uint8_t>(index & 0xFF);
  message.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  return message;
}

can_msgs::msg::Frame Protocol::make_enable_frame(uint8_t motor_id)
{
  return make_base_frame(TYPE_ENABLE, motor_id);
}

can_msgs::msg::Frame Protocol::make_reset_frame(uint8_t motor_id)
{
  return make_base_frame(TYPE_RESET, motor_id);
}


} // namespace edulite05_driver
