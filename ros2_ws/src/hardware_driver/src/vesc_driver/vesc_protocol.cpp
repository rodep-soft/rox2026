#include "vesc_driver/vesc_protocol.hpp"
namespace vesc_driver::protocol
{namespace
{
int32_t i32(const std::array<uint8_t, 8> & d)
{
  return static_cast<int32_t>((uint32_t(d[0]) <<
         24) | (uint32_t(d[1]) << 16) | (uint32_t(d[2]) << 8) | d[3]);
}
int16_t i16(const std::array<uint8_t, 8> & d, size_t i)
{
  return static_cast<int16_t>((uint16_t(d[i]) << 8) | d[i + 1]);
}
}
can_msgs::msg::Frame make_set_rpm_frame(uint8_t id, int32_t erpm)
{
  can_msgs::msg::Frame f{};f.id = (SET_RPM << 8) | id;f.is_extended = true;f.dlc = 4;
  auto v = uint32_t(erpm);for (int i = 0; i < 4; i++) {
    f.data[i] = (v >> (24 - 8 * i)) & 0xff;
  }
  return f;
}
bool decode_status_1(const can_msgs::msg::Frame & f, Status1 & s)
{
  if (!f.is_extended || f.is_rtr || f.is_error || f.dlc != 8 || (f.id >> 8) != STATUS_1) {
    return false;
  }
  s.controller_id = f.id & 0xff;s.erpm = i32(f.data);s.motor_current = i16(f.data, 4) / 10.0F;
  s.duty_cycle = i16(f.data, 6) / 1000.0F;return true;
}
}
