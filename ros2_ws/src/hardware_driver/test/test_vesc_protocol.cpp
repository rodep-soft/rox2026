#include <gtest/gtest.h>

#include "vesc_driver/vesc_protocol.hpp"

TEST(VescProtocol, EncodesPositiveRpm)
{
  const auto frame = vesc_driver::protocol::make_set_rpm_frame(1U, 3000);

  EXPECT_EQ(frame.id, 0x301U);
  EXPECT_TRUE(frame.is_extended);
  EXPECT_EQ(frame.dlc, 4U);
  EXPECT_EQ(frame.data[0], 0x00U);
  EXPECT_EQ(frame.data[1], 0x00U);
  EXPECT_EQ(frame.data[2], 0x0bU);
  EXPECT_EQ(frame.data[3], 0xb8U);
}

TEST(VescProtocol, EncodesNegativeRpm)
{
  const auto frame = vesc_driver::protocol::make_set_rpm_frame(42U, -3000);

  EXPECT_EQ(frame.id, 0x32aU);
  EXPECT_EQ(frame.data[0], 0xffU);
  EXPECT_EQ(frame.data[1], 0xffU);
  EXPECT_EQ(frame.data[2], 0xf4U);
  EXPECT_EQ(frame.data[3], 0x48U);
}

TEST(VescProtocol, DecodesStatus1Erpm)
{
  can_msgs::msg::Frame frame{};
  frame.id = 0x92aU;
  frame.is_extended = true;
  frame.dlc = 8U;
  frame.data = {0U, 0U, 0x0bU, 0xb8U, 0U, 0x7bU, 1U, 0xf4U};

  vesc_driver::protocol::Status1 status{};
  ASSERT_TRUE(vesc_driver::protocol::decode_status_1(frame, status));
  EXPECT_EQ(status.controller_id, 42U);
  EXPECT_EQ(status.erpm, 3000);
}

TEST(VescProtocol, RejectsNonStatus1Frames)
{
  can_msgs::msg::Frame frame{};
  frame.id = 0x82aU;
  frame.is_extended = true;
  frame.dlc = 8U;

  vesc_driver::protocol::Status1 status{};
  EXPECT_FALSE(vesc_driver::protocol::decode_status_1(frame, status));

  frame.id = 0x92aU;
  frame.dlc = 4U;
  EXPECT_FALSE(vesc_driver::protocol::decode_status_1(frame, status));
}
