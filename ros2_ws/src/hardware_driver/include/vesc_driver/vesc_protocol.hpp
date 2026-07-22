#pragma once
#include <cstdint>
#include "can_msgs/msg/frame.hpp"
namespace vesc_driver::protocol
{
constexpr uint32_t SET_RPM = 3U, STATUS_1 = 9U;
struct Status1 {uint8_t controller_id; int32_t erpm; float motor_current; float duty_cycle;};
can_msgs::msg::Frame make_set_rpm_frame(uint8_t id, int32_t erpm);
bool decode_status_1(const can_msgs::msg::Frame & frame, Status1 & status);
}
