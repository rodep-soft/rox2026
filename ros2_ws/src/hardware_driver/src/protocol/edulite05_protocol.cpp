#include "edulite05_driver/edulite05_protocol.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

Ed05CanframeCreater::Ed05CanframeCreater(uint8_t motor_id)
: motor_id_(motor_id)
{
}

Velocity::Velocity(uint8_t motor_id)
: Ed05CanframeCreater(motor_id)
{
}

Position::Position(uint8_t motor_id)
: Ed05CanframeCreater(motor_id)
{
}

std::vector<Canframe> Velocity::create_init_frame()
{
  std::vector<Canframe> frames;

  // runmode 
  frames.push_back(set_runmode(2)); // runmode: velocity

  // motor enable
  frames.push_back(set_enable());

  // その他制御量の設定
  for (size_t i = 1; i < targets_info.size(); i++) {
    frames.push_back(set_target_value(targets_info[i]));
  }

  return frames;
}

std::vector<Canframe> Position::create_init_frame()
{
  std::vector<Canframe> frames;

  // disable
  frames.push_back(set_disable());
  // runmode: operation
  frames.push_back(set_runmode(0)); // runmode: operation
  // enable
  frames.push_back(set_enable());
  // mechanical zero
  frames.push_back(set_mechanicalzero());
  frames.push_back(set_disable());
  frames.push_back(set_runmode(1)); // runmode: position
  frames.push_back(set_enable());
  for (size_t i = 1; i < targets_info.size(); i++) {
    frames.push_back(set_target_value(targets_info[i]));
  }
  frames.push_back(set_angle_range()); // angleを-180~180にするための設定

  return frames;
}

Canframe Velocity::create_control_frame(float value)
{
  targets_info[0].value = std::clamp(value, -50.0f, 50.0f);
  return set_target_value(targets_info[0]);
}

Canframe Position::create_control_frame(float value)
{
  targets_info[0].value = std::clamp(value, -static_cast<float>(M_PI), static_cast<float>(M_PI));
  return set_target_value(targets_info[0]);
}

uint32_t Ed05CanframeCreater::encode_can_id(uint8_t commtype_index)
{
  return (static_cast<uint32_t>(commtype_index) << 24) |
         (static_cast<uint32_t>(host_id_) << 8) |
         static_cast<uint32_t>(motor_id_);
}

std::array<uint8_t, 8> Ed05CanframeCreater::encode_commtype18_data(ControlTargetInfo target_info)
{
  std::array<uint8_t, 8> data{};
  uint16_t index = target_info.index;
  data[0] = static_cast<uint8_t>(index & 0xFF);
  data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  data[2] = 0;
  data[3] = 0;

  uint32_t val_bits = 0;
  std::memcpy(&val_bits, &target_info.value, sizeof(val_bits));
  data[4] = static_cast<uint8_t>(val_bits & 0xFF);
  data[5] = static_cast<uint8_t>((val_bits >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((val_bits >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((val_bits >> 24) & 0xFF);

  return data;
}

Canframe Ed05CanframeCreater::set_runmode(int value)
{
    Canframe frame{};
    frame.id = encode_can_id(0x12);
    frame.dlc = dlc_;
    //ControlTargetInfo info{"runmode", 0x7005, static_cast<float>(value)};
    //auto data = encode_commtype18_data(info);
    //std::memcpy(frame.data.data(), data.data(), 8);
    frame.data[0] = 0x05; // index low byte
    frame.data[1] = 0x70; // index high byte
    frame.data[2] = 0x00; // subindex 
    frame.data[3] = 0x00; // reserved
    frame.data[4] = static_cast<uint8_t>(value & 0xFF); // value low byte
    frame.data[5] = 0x00; // value high byte
    frame.data[6] = 0x00; // reserved
    frame.data[7] = 0x00; // reserved
    return frame;
}

Canframe Ed05CanframeCreater::set_enable()
{
  Canframe frame{};
  frame.id = encode_can_id(0x03);
  frame.dlc = dlc_;
  frame.data.fill(0);
  return frame;
}

Canframe Ed05CanframeCreater::set_target_value(ControlTargetInfo target_info)
{
  Canframe frame{};
  frame.id = encode_can_id(0x12);
  frame.dlc = dlc_;
  auto data = encode_commtype18_data(target_info);
  std::memcpy(frame.data.data(), data.data(), 8);
  return frame;
}

Canframe Ed05CanframeCreater::set_disable()
{
  Canframe frame{};
  frame.id = encode_can_id(0x04);
  frame.dlc = dlc_;
  frame.data.fill(0);
  return frame;
}

Canframe Ed05CanframeCreater::set_mechanicalzero()
{
  Canframe frame{};
  frame.id = encode_can_id(0x06);
  frame.dlc = dlc_;
  frame.data.fill(0);
  return frame;
}

Canframe Ed05CanframeCreater::terminate_motor()
{
  return set_disable();
}

Canframe Ed05CanframeCreater::set_angle_range()
{
  Canframe frame{};
  frame.id = encode_can_id(0x12);
  frame.dlc = dlc_;
    frame.data[0] = 0x29; // index low byte
    frame.data[1] = 0x70; // index high byte
    frame.data[2] = 0x00; // subindex 
    frame.data[3] = 0x00; // reserved
    frame.data[4] = 0x01; // value low byte
    frame.data[5] = 0x00; // value high byte
    frame.data[6] = 0x00; // reserved
    frame.data[7] = 0x00; // reserved
  return frame;
}
