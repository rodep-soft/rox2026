#include <gtest/gtest.h>
#include "vesc_driver/vesc_protocol.hpp"
TEST(VescProtocol, EncodeRpm) {
  const auto p = vesc_driver::protocol::make_set_rpm_frame(1U, 3000);
  EXPECT_EQ(p.id, 0x301U); EXPECT_TRUE(p.is_extended); EXPECT_EQ(p.dlc, 4U);
  EXPECT_EQ(p.data[2], 0x0bU); EXPECT_EQ(p.data[3], 0xb8U);
  const auto n = vesc_driver::protocol::make_set_rpm_frame(42U, -3000);
  EXPECT_EQ(n.data[0], 0xffU); EXPECT_EQ(n.data[2], 0xf4U); EXPECT_EQ(n.data[3], 0x48U);
}
TEST(VescProtocol, DecodeStatus) {
  can_msgs::msg::Frame f{}; f.id = 0x92aU; f.is_extended = true; f.dlc = 8U;
  f.data = {0U, 0U, 0x0bU, 0xb8U, 0U, 0x7bU, 1U, 0xf4U};
  vesc_driver::protocol::Status1 s{};
  ASSERT_TRUE(vesc_driver::protocol::decode_status_1(f, s));
  EXPECT_EQ(s.controller_id, 42U); EXPECT_EQ(s.erpm, 3000);
  EXPECT_FLOAT_EQ(s.motor_current, 12.3F); EXPECT_FLOAT_EQ(s.duty_cycle, 0.5F);
}
