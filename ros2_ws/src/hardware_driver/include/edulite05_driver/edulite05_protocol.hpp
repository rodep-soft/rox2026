//#pragma once

#include <stdint.h>
#include <array>
#include <vector>
#include <memory>
#include <string>

struct Canframe
{
  uint32_t id;
  int dlc;
  std::array<uint8_t, 8> data;
};

class Ed05CanframeCreater
{
public:
  Ed05CanframeCreater(uint8_t motor_id);
  virtual ~Ed05CanframeCreater() = default;

  virtual std::vector<Canframe> create_init_frame() = 0;
  virtual Canframe create_control_frame(float value) = 0;
  Canframe terminate_motor();

protected:
  struct ControlTargetInfo
  {
    char name[12];
    uint16_t index;     // commtype18で使用
    float value;
  };

  uint8_t motor_id_;
  int dlc_ = 8;
  uint8_t host_id_ = 0xFD;

  uint32_t encode_can_id(uint8_t commtype_index);
  std::array<uint8_t, 8> encode_commtype18_data(ControlTargetInfo target_info);

  Canframe set_runmode(int value);
  Canframe set_enable();
  Canframe set_target_value(ControlTargetInfo target_info);
  Canframe set_disable();
  Canframe set_mechanicalzero();
};

class Velocity : public Ed05CanframeCreater
{
public:
  explicit Velocity(uint8_t motor_id);
  std::vector<Canframe> create_init_frame() override;
  Canframe create_control_frame(float value) override;

private:
    std::array<ControlTargetInfo, 3> targets_info = {{
        {"vel", 0x700A, 0.0f},
        {"limit_cur", 0x7018, 5.0f},
        {"acc", 0x7022, 100.0f},
    }};
};

class Position : public Ed05CanframeCreater
{
public:
  explicit Position(uint8_t motor_id);
  std::vector<Canframe> create_init_frame() override;
  Canframe create_control_frame(float value) override;

private:
  std::array<ControlTargetInfo, 5> targets_info = {{
    {"loc_ref", 0x7016, 0.0f},
    {"limit_cur", 0x7018, 5.0f},
    {"vel_max", 0x7024, 50.0f},
    {"acc_set", 0x7025, 100.0f},
    {"zero_sta", 0x7029, 1.0f},
  }};
};
