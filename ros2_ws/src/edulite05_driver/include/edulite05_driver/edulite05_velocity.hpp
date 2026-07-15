#pragma once

#include <chrono>
#include <cstdint>
#include <thread>

#include "edulite05_driver/edulite05_protocol.hpp"

// EDULITE05のCAN通信実装（1モータ分）。
class EduLite05Velocity final : public EduLite05Protocol
{
public:
  explicit EduLite05Velocity(
    std::uint8_t motor_id,
    const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr & can_publisher,
    float current_limit_a = 11.0F,
    float acceleration_rad_s2 = 20.0F)
  : EduLite05Protocol(motor_id, can_publisher),
    current_limit_a_(current_limit_a),
    acceleration_rad_s2_(acceleration_rad_s2)
  {
  }

  void initialize() override
  {
    // Communication type 18: run_mode (0x7005) = 2, Velocity mode
    write_u32(kIndexRunMode, kVelocityMode);
    wait_after_command();

    // Communication type 3: motor enabled to run
    publish(make_frame(kCommEnable));
    wait_after_command();

    // Communication type 18: current_limit (0x7006)
    write_float(kIndexCurrentLimit, current_limit_a_);
    wait_after_command();

    // Communication type 18: velocity ramp acceleration (0x7022)
    write_float(kIndexVelocityAcceleration, acceleration_rad_s2_);
    wait_after_command();
  }

  void set_velocity(float velocity_rad_s) override
  {
    write_float(kIndexVelocityCommand, velocity_rad_s);
  }

  void stop() override
  {
    set_velocity(0.0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    publish(make_frame(kCommStop));
  }

private:
  static constexpr std::uint8_t kCommEnable = 0x03;
  static constexpr std::uint8_t kCommStop = 0x04;
  static constexpr std::uint8_t kCommSingleParameterWrite = 0x12;

  static constexpr std::uint16_t kIndexRunMode = 0x7005;
  static constexpr std::uint16_t kIndexCurrentLimit = 0x7006;
  static constexpr std::uint16_t kIndexVelocityCommand = 0x700A;
  static constexpr std::uint16_t kIndexVelocityAcceleration = 0x7022;
  static constexpr std::uint32_t kVelocityMode = 2;

  void write_u32(std::uint16_t index, std::uint32_t value)
  {
    auto frame = make_frame(kCommSingleParameterWrite);
    put_index(frame, index);
    frame.data[4] = static_cast<std::uint8_t>(value & 0xFFU);
    frame.data[5] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
    frame.data[6] = static_cast<std::uint8_t>((value >> 16) & 0xFFU);
    frame.data[7] = static_cast<std::uint8_t>((value >> 24) & 0xFFU);
    publish(std::move(frame));
  }

  void write_float(std::uint16_t index, float value)
  {
    auto frame = make_frame(kCommSingleParameterWrite);
    put_index(frame, index);
    put_float_le(frame, 4, value);
    publish(std::move(frame));
  }

  static void wait_after_command()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  float current_limit_a_;
  float acceleration_rad_s2_;
};
