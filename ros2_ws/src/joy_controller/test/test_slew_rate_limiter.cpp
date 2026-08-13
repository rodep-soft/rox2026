#include <gtest/gtest.h>

#include "joy_controller/slew_rate_limiter.hpp"

TEST(SlewRateLimiterTest, LimitsAccelerationAndDeceleration)
{
  joy_controller::SlewRateLimiter limiter(2.0, 4.0);

  EXPECT_DOUBLE_EQ(limiter.update(10.0, 0.1), 0.2);
  EXPECT_DOUBLE_EQ(limiter.update(10.0, 0.1), 0.4);
  EXPECT_DOUBLE_EQ(limiter.update(0.0, 0.05), 0.2);
  EXPECT_NEAR(limiter.update(0.0, 0.05), 0.0, 1e-12);
}

TEST(SlewRateLimiterTest, StopsBeforeReversingDirection)
{
  joy_controller::SlewRateLimiter limiter(2.0, 4.0);
  limiter.reset(0.3);

  EXPECT_DOUBLE_EQ(limiter.update(-1.0, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(limiter.update(-1.0, 0.1), -0.2);
}

TEST(SlewRateLimiterTest, ResetBypassesRamp)
{
  joy_controller::SlewRateLimiter limiter(1.0, 1.0);
  limiter.update(1.0, 0.1);
  limiter.reset();

  EXPECT_DOUBLE_EQ(limiter.value(), 0.0);
}
