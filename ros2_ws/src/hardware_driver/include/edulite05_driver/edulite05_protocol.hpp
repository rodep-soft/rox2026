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

struct CanIdInfo
{
  uint8_t comm_type;   // Bit 28~24
  uint8_t mode_status; // Bit 23~22
  uint8_t fault_info;  // Bit 21~16
  uint8_t motor_id;    // Bit 15~8
  uint8_t host_id;     // Bit 7~0
};

struct MotorFeedbackData
{
  float angle;        // [rad] (-4pi ~ 4pi)
  float velocity;     // [rad/s] (-50 ~ 50)
  float torque;       // [Nm] (-6 ~ 6)
  float temperature;  // [Celsius]
};

CanIdInfo decode_can_id(uint32_t can_id);
MotorFeedbackData decode_feedback_data(const std::array<uint8_t, 8> & data);


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
  Canframe set_angle_range(); // angleを-180~180にするための設定
};

class Velocity : public Ed05CanframeCreater
{
public:
  explicit Velocity(uint8_t motor_id);
  std::vector<Canframe> create_init_frame() override;
  Canframe create_control_frame(float value) override;

private:
  std::array<ControlTargetInfo, 3> targets_info = {{
    {"vel", 0x700A, 0.0f},     // 速度指令
    {"limit_cur", 0x7018, 5.0f},     // 電流制限
    {"acc_rad", 0x7022, 100.0f},     // 加速度制限
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
    {"loc_ref", 0x7016, 0.0f}, // 位置指令
    {"limit_cur", 0x7018, 5.0f}, // 電流制限
    {"vel_max", 0x7024, 50.0f},  // 速度制限
    {"acc_set", 0x7025, 100.0f}, // 加速度
    //{"zero_sta", 0x7029, 1.0f},  //
  }};
};
