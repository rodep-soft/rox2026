#include "edulite05_driver/edulite05_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace edulite05_driver
{

namespace
{
  uint16_t read_be_u16(const std::array<uint8_t, 8> & data, size_t index)
  {
    return (static_cast<uint16_t>(data[index]) << 8) | static_cast<uint16_t>(data[index + 1]);
  }

  float decode_u16(uint16_t value, float min, float max)
  {
    return min + static_cast<float>(value) * (max - min) / 65535.0f;
  }
}

Protocol::Protocol(const MotorConfig & config) : config_(config),position_offset_(config.position_offset)
{
  homed_ = !config_.require_homing;
  init_items_.clear();
  // 最初に必ずrun_mode
  init_items_.push_back({RUN_MODE, static_cast<float>(static_cast<uint8_t>(config_.mode)),true});

  switch (config_.mode) {
    case Mode::VELOCITY:
      init_items_.push_back({LIMIT_CURRENT, config_.current_limit,false});
      init_items_.push_back({ACCELERATION, config_.acceleration,false});
      break;
    case Mode::CSP:
      init_items_.push_back({LIMIT_SPEED,config_.speed_limit,false});
      init_items_.push_back({LIMIT_CURRENT,config_.current_limit,false});
      break;
    case Mode::PP:
      init_items_.push_back({PP_SPEED,config_.speed_limit,false});
      init_items_.push_back({PP_ACCELERATION,config_.acceleration,false});
      init_items_.push_back({LIMIT_CURRENT,config_.current_limit,false});
      break;
  }
}

void Protocol::set_target(float target)
{
  target_ = target;
  target_received_ = true;
  last_target_time_ = Clock::now();
}

bool Protocol::set_position_offset(float offset)
{
  if (!connected_) {
    return false;
  }
  position_offset_ = offset - raw_position_;
  feedback_.position = offset;

  homed_ = true;
  return true;
}

std::optional<can_msgs::msg::Frame> Protocol::create_initialization_frame()
{
  const auto now = Clock::now();
  switch (init_step_) {
    case InitStep::WRITE_ITEM:
    {
      state_ = MotorState::INITIALIZING;
      const auto & item = init_items_[init_index_];
      last_request_time_ = now;
      init_step_ = InitStep::WAIT_WRITE;

      if (item.is_u8) {
        return create_write_u8_frame(config_.can_id,item.index,static_cast<uint8_t>(item.value));
      }
      return create_write_float_frame(config_.can_id,item.index,item.value);
    }
    case InitStep::WAIT_WRITE:
    {
      // これで帰ってきた応答そのものを設定成功判定には使わずに，type17 Readbackで確認
      if (now - last_request_time_ >= WRITE_WAIT) {
        init_step_ = InitStep::READ_ITEM;
      }
      break;
    }
    case InitStep::READ_ITEM:
    {
      const auto & item = init_items_[init_index_];
      last_request_time_ = now;
      init_step_ = InitStep::WAIT_READ;
      return create_read_parameter_frame(config_.can_id, item.index);
    }
    case InitStep::WAIT_READ:
    {
      if (now - last_request_time_ > RESPONSE_TIMEOUT)
      {
        retry_initialization();
      }
      break;
    }
    case InitStep::ENABLE:
    {
      last_request_time_ = now;
      init_step_ = InitStep::WAIT_ENABLE;
      return create_enable_frame(config_.can_id);
    }
    case InitStep::WAIT_ENABLE:
    {
      if (now - last_request_time_ > RESPONSE_TIMEOUT)
      {
        retry_initialization();
      }
      break;
    }
    case InitStep::READY:
      break;
    case InitStep::ERROR:
    {
      // 電源再投入などでもROSノードを再起動しなくて済むように自動再試行
      if (now - error_time_ > ERROR_RETRY_PERIOD)
      {
        restart(true);
      }
      break;
    }
  }
  return std::nullopt;
}

bool Protocol::receive(const can_msgs::msg::Frame & msg)
{
  if (!msg.is_extended || msg.dlc < 8) {
    return false;
  }

  const uint8_t type = static_cast<uint8_t>((msg.id >> 24) & 0x1F);
  const uint8_t motor_id = static_cast<uint8_t>((msg.id >> 8) & 0xFF);

  if (motor_id != config_.can_id) {
    return false;
  }
  if (type == TYPE_FEEDBACK) {
    process_feedback(msg);
    return true;
  }
  if (type == TYPE_READ) {
    process_parameter_response(msg);
  }
  last_rx_time_ = Clock::now();
  return false;
}

void Protocol::process_feedback(const can_msgs::msg::Frame & msg)
{
  connected_ = true;
  last_rx_time_ = Clock::now();
  raw_position_ = decode_u16(read_be_u16(msg.data, 0),-4.0f * PI, 4.0f * PI);
  feedback_.position = raw_position_ + position_offset_;
  feedback_.velocity = decode_u16(read_be_u16(msg.data, 2),-50.0f ,50.0f);
  feedback_.effort = decode_u16(read_be_u16(msg.data, 4), -6.0f, 6.0f);
  feedback_.temperature = static_cast<float>(read_be_u16(msg.data, 6)) / 10.0f;
  // Type2 ID bit21~16
  feedback_.fault_code = static_cast<uint32_t>((msg.id >> 16) & 0x3F);
  // bit23~22
  const uint8_t mode_status = static_cast<uint8_t>((msg.id >> 22) & 0x03);
  // Enable完了確認
  if (init_step_ == InitStep::WAIT_ENABLE) {
    if (mode_status == RUN_STATUS_MODE) {
      enabled_ = true;
      retry_count_ = 0;
      if (init_index_ >= init_items_.size()) {
        configured_ = true;
        state_ = MotorState::READY;
        init_step_ = InitStep::READY;
      } else {
        init_step_ = InitStep::WRITE_ITEM;
      }
    }
    return;
  }

  // 動作中にResetへ戻った場合
  if (init_step_ == InitStep::READY && mode_status != RUN_STATUS_MODE)
  {
    restart(true);
  }
}

void Protocol::process_parameter_response(const can_msgs::msg::Frame & msg)
{
  connected_ = true;
  last_rx_time_ = Clock::now();
  if (init_step_ != InitStep::WAIT_READ) {
    return;
  }
  const uint8_t destination = static_cast<uint8_t>(msg.id & 0xFF);
  const uint8_t status = static_cast<uint8_t>((msg.id >> 16) & 0xFF);
  if (destination != HOST_ID) {
    return;
  }
  // Type17 status != 0
  if (status != RESET_STATUS_MODE) {
    retry_initialization();
    return;
  }

  const uint16_t index = static_cast<uint16_t>(msg.data[0]) 
  | (static_cast<uint16_t>(msg.data[1]) << 8);

  const auto & expected =init_items_[init_index_];
  // 古い別parameterの応答などは無視
  if (index != expected.index) {
    return;
  }

  bool matched = false;
  if (expected.is_u8) {
    matched = msg.data[4] == static_cast<uint8_t>(expected.value);
  } else {
    float value = 0.0f;
    std::memcpy(&value, msg.data.data() + 4, sizeof(float));
    matched = std::fabs(value - expected.value) < 0.001f;
  }
  if (!matched) {
    retry_initialization();
    return;
  }

  retry_count_ = 0;
  ++init_index_;

  if (init_index_ >= init_items_.size()) {
    configured_ = enabled_;
    state_ = MotorState::READY;
    init_step_ = InitStep::READY;
  } else if (!enabled_) {
    init_step_ = InitStep::ENABLE;
  } else {
    init_step_ = InitStep::WRITE_ITEM;
  }
}

void Protocol::retry_initialization()
{
  ++retry_count_;
  if (retry_count_ > MAX_RETRY) {
    state_ = MotorState::ERROR;
    configured_ = false;
    enabled_ = false;
    init_step_ = InitStep::ERROR;
    error_time_ = Clock::now();
    return;
  }
  switch (init_step_) {
    case InitStep::WAIT_READ:
      // Writeからやり直す
      init_step_ = InitStep::WRITE_ITEM;
      break;
    case InitStep::WAIT_ENABLE:
      init_step_ = InitStep::ENABLE;
      break;
    default:
      init_step_ = InitStep::WRITE_ITEM;
      break;
  }
}

void Protocol::restart(bool clear_target)
{
  state_ = MotorState::INITIALIZING;
  configured_ = false;
  enabled_ = false;
  init_index_ = 0;
  retry_count_ = 0;
  init_step_ = InitStep::WRITE_ITEM;
  if (clear_target) {
    target_received_ = false;
    target_ = 0.0f;
  }
}

std::optional<can_msgs::msg::Frame> Protocol::create_target_frame()
{
  if (state_ != MotorState::READY || !target_received_)
  {
    return std::nullopt;
  }
   if ((config_.mode == Mode::PP || config_.mode == Mode::CSP) && !homed_)
  {
    return std::nullopt;
  }

  const auto now = Clock::now();

  if (last_command_time_ != TimePoint{} && now - last_target_time_ < std::chrono::milliseconds(config_.command_period_ms)) {
    return std::nullopt;
  }

  last_command_time_ = now;
  float target = target_;

  if (config_.mode == Mode::VELOCITY) {
    const bool timeout = now - last_target_time_ > std::chrono::milliseconds(config_.target_timeout_ms);
    // 上位制御ノードが死んだ場合は停止
    if (timeout) {
      target = 0.0f;
    }
    target = std::clamp(target, -50.0f, 50.0f);
    return create_write_float_frame(config_.can_id, SPEED_REF, target);
  }

  // PP / CSPは指令が途絶えても最後の位置を保持する
  target = std::clamp(target_,config_.position_min, config_.position_max) - position_offset_;

  return create_write_float_frame(config_.can_id, POSITION_REF, target);
}

void Protocol::watchdog()
{
  if (state_ != MotorState::READY || !target_received_ || !connected_)
  {
    return;
  }
  const auto now = Clock::now();

  if (now - last_rx_time_ <= std::chrono::milliseconds(config_.feedback_timeout_ms))
  {
    return;
  }
  connected_ = false;
  // 再接続後に古いtargetで突然動かないように
  // targetも破棄
  restart(true);
}

can_msgs::msg::Frame Protocol::make_base_frame(uint8_t type, uint8_t motor_id)
{
  can_msgs::msg::Frame msg;
  msg.id = (static_cast<uint32_t>(type) << 24) | (static_cast<uint32_t>(HOST_ID) << 8) | motor_id;
  msg.is_extended = true;
  msg.dlc = 8;
  msg.data.fill(0);
  return msg;
}

can_msgs::msg::Frame Protocol::create_write_u8_frame(uint8_t motor_id, uint16_t index, uint8_t value)
{
  auto msg = make_base_frame(TYPE_WRITE, motor_id);
  msg.data[0] = static_cast<uint8_t>(index & 0xFF);
  msg.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  msg.data[4] = value;
  return msg;
}

can_msgs::msg::Frame Protocol::create_write_float_frame(uint8_t motor_id, uint16_t index, float value)
{
  auto msg = make_base_frame(TYPE_WRITE, motor_id);
  msg.data[0] = static_cast<uint8_t>(index & 0xFF);
  msg.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  std::memcpy(msg.data.data() + 4, &value, sizeof(float));
  return msg;
}

can_msgs::msg::Frame Protocol::create_read_parameter_frame(uint8_t motor_id, uint16_t index)
{
  auto msg = make_base_frame(TYPE_READ, motor_id);
  msg.data[0] = static_cast<uint8_t>(index & 0xFF);
  msg.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  return msg;
}

can_msgs::msg::Frame Protocol::create_enable_frame(uint8_t motor_id)
{
  return make_base_frame(TYPE_ENABLE, motor_id);
}
}  // namespace edulite05_driver
