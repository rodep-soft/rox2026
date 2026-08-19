#include <gtest/gtest.h>

#include <cmath>

#include "libbno055-linux/controllers/heading_controller.hpp"

namespace bno055lib::test {

TEST(HeadingControllerTest, AngleNormalization) {
    EXPECT_NEAR(normalizeAngleDeg(0.0), 0.0, 1e-6);
    EXPECT_NEAR(normalizeAngleDeg(90.0), 90.0, 1e-6);
    EXPECT_NEAR(normalizeAngleDeg(180.0), 180.0, 1e-6);
    EXPECT_NEAR(normalizeAngleDeg(-180.0), -180.0, 1e-6);
    EXPECT_NEAR(normalizeAngleDeg(360.0), 0.0, 1e-6);
    EXPECT_NEAR(normalizeAngleDeg(370.0), 10.0, 1e-6);
    EXPECT_NEAR(normalizeAngleDeg(-370.0), -10.0, 1e-6);
    EXPECT_NEAR(std::abs(normalizeAngleDeg(540.0)), 180.0, 1e-6);
}

TEST(HeadingControllerTest, FastYawExtraction) {
    // Identity quaternion (0 degrees yaw)
    EXPECT_NEAR(fastExtractYawDeg(1.0, 0.0, 0.0, 0.0), 0.0, 1e-4);

    // 90 degrees Yaw around Z (w = cos(45°), z = sin(45°))
    constexpr double q90_w = 0.7071067811865476;
    constexpr double q90_z = 0.7071067811865476;
    EXPECT_NEAR(fastExtractYawDeg(q90_w, 0.0, 0.0, q90_z), 90.0, 1e-4);

    // -90 degrees Yaw around Z
    EXPECT_NEAR(fastExtractYawDeg(q90_w, 0.0, 0.0, -q90_z), -90.0, 1e-4);

    // 180 degrees Yaw around Z (w = 0, z = 1)
    EXPECT_NEAR(std::abs(fastExtractYawDeg(0.0, 0.0, 0.0, 1.0)), 180.0, 1e-4);
}

TEST(HeadingControllerTest, ShortestPathHeadingError) {
    HeadingController controller;

    // Target = 10 deg, Current = 0 deg -> Error = +10 deg
    auto out1 = controller.update(10.0, 0.0, 0.01);
    EXPECT_NEAR(out1.error_deg, 10.0, 1e-6);

    // Target = -170 deg, Current = +170 deg -> Shortest Error = +20 deg (wrap across +-180)
    auto out2 = controller.update(-170.0, 170.0, 0.01);
    EXPECT_NEAR(out2.error_deg, 20.0, 1e-6);

    // Target = +170 deg, Current = -170 deg -> Shortest Error = -20 deg
    auto out3 = controller.update(170.0, -170.0, 0.01);
    EXPECT_NEAR(out3.error_deg, -20.0, 1e-6);
}

TEST(HeadingControllerTest, ProportionalResponse) {
    HeadingController::Config cfg;
    cfg.kp = 0.1;
    cfg.ki = 0.0;
    cfg.kd = 0.0;
    cfg.min_output = -1.0;
    cfg.max_output = 1.0;

    HeadingController controller(cfg);

    // Target = 10 deg, Current = 0 deg -> Correction = 0.1 * 10 = 1.0
    auto out = controller.update(10.0, 0.0, 0.01, 0.0, 0.5);
    EXPECT_NEAR(out.correction, 1.0, 1e-6);
    EXPECT_NEAR(out.left_motor, 0.0, 1.e-6);   // 0.5 - 1.0 = -0.5 clamped to 0.0
    EXPECT_NEAR(out.right_motor, 1.0, 1.e-6);  // 0.5 + 1.0 = 1.5 clamped to 1.0
}

TEST(HeadingControllerTest, AntiWindupIntegralClamping) {
    HeadingController::Config cfg;
    cfg.kp = 0.0;
    cfg.ki = 1.0;
    cfg.kd = 0.0;
    cfg.max_i_term = 0.2;  // Clamp I term to +-0.2

    HeadingController controller(cfg);

    // Run 50 iterations with persistent 10 deg error (dt = 0.01)
    HeadingController::Output out;
    for (int i = 0; i < 50; ++i) {
        out = controller.update(10.0, 0.0, 0.01);
    }

    // Accumulated integral should be clamped at 0.2 instead of 0.01 * 10 * 50 = 5.0
    EXPECT_NEAR(out.correction, 0.2, 1e-6);
}

TEST(HeadingControllerTest, GyroBasedDerivativeControl) {
    HeadingController::Config cfg;
    cfg.kp = 0.0;
    cfg.ki = 0.0;
    cfg.kd = 0.05;  // Kd = 0.05

    HeadingController controller(cfg);

    // Target = 0 deg, Current = 0 deg, Gyro Yaw Rate = +20 deg/s (spinning right)
    // Derivative term = -Kd * gyro_z = -0.05 * 20 = -1.0 (apply left counter-torque)
    auto out = controller.update(0.0, 0.0, 0.01, 20.0);
    EXPECT_NEAR(out.correction, -1.0, 1e-6);
}

TEST(HeadingControllerTest, DifferentialWheelSpeedClamping) {
    HeadingController controller;

    // Zero correction, base_velocity = 0.6
    auto out = controller.update(0.0, 0.0, 0.01, 0.0, 0.6);
    EXPECT_NEAR(out.left_motor, 0.6, 1e-6);
    EXPECT_NEAR(out.right_motor, 0.6, 1e-6);
}

TEST(HeadingControllerTest, DirectQuaternionUpdate) {
    HeadingController controller;

    // Target Quat: Identity (0 deg)
    Quat q_target{1.0, 0.0, 0.0, 0.0};
    // Current Quat: 90 deg yaw (w=0.7071, z=0.7071) -> error = -90 deg
    Quat q_current{0.7071067811865476, 0.0, 0.0, 0.7071067811865476};

    auto out = controller.update(q_target, q_current, 0.01);
    EXPECT_NEAR(out.error_deg, -90.0, 1e-3);
}

TEST(HeadingControllerTest, SlewRateLimiting) {
    HeadingController::Config cfg;
    cfg.kp = 1.0;
    cfg.max_slew_rate = 2.0;  // Max change rate 2.0 rad/s^2

    HeadingController controller(cfg);

    // Initial step: raw correction would be P * error = 1.0 * 10.0 = 10.0 (clamped to max_output 1.0)
    // First step: initialized -> output = 1.0
    auto out1 = controller.update(10.0, 0.0, 0.1);
    EXPECT_NEAR(out1.correction, 1.0, 1e-6);

    // Second step: target jumps to -10.0 -> raw output = -1.0 (delta = -2.0)
    // Max change per 0.1s is max_slew_rate * dt = 2.0 * 0.1 = 0.2
    // So output should change from 1.0 to 1.0 - 0.2 = 0.8
    auto out2 = controller.update(-10.0, 0.0, 0.1);
    EXPECT_NEAR(out2.correction, 0.8, 1e-6);
}

TEST(HeadingControllerTest, QuaternionValidation) {
    Quat valid_q{1.0, 0.0, 0.0, 0.0};
    Quat nan_q{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0};
    Quat corrupted_q{5.0, 5.0, 5.0, 5.0};  // Norm squared = 100 != 1.0

    EXPECT_TRUE(isValidQuat(valid_q));
    EXPECT_FALSE(isValidQuat(nan_q));
    EXPECT_FALSE(isValidQuat(corrupted_q));
}

}  // namespace bno055lib::test
