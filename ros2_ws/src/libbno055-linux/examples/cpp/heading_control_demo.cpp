#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "libbno055-linux/bno055.hpp"
#include "libbno055-linux/controllers/heading_controller.hpp"

constexpr uint8_t DEFAULT_ADDR = 0x28;
constexpr auto LOOP_PERIOD = std::chrono::microseconds(10000);  // 100Hz ultra-fast loop (10ms)

/**
 * @brief Zero-allocation ASCII bar generator using static stack buffer.
 */
inline void renderBarToBuffer(char* dest, size_t dest_size, double val, int width = 20) noexcept {
    val = std::clamp(val, -1.0, 1.0);
    int center = width / 2;
    int pos = center + static_cast<int>(val * center);
    pos = std::clamp(pos, 0, width - 1);

    if (width >= static_cast<int>(dest_size)) width = static_cast<int>(dest_size) - 1;
    std::memset(dest, ' ', width);
    dest[center] = '|';

    if (pos >= center) {
        std::memset(dest + center + 1, '=', pos - center);
    } else {
        std::memset(dest + pos, '=', center - pos);
    }
    dest[width] = '\0';
}

int main(int argc, char* argv[]) {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);

    std::string device = "/dev/i2c-1";
    if (argc > 1) device = argv[1];

    std::cout << "Initializing BNO055 [Production 100Hz Heading PID Demo] on " << device << "...\n";

    bno055lib::BNO055 imu(DEFAULT_ADDR, device);
    if (!imu.begin(bno055lib::OpMode::NDOF)) {
        std::cerr << "Failed to initialize BNO055!\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Lock initial target heading
    double target_heading_deg = 0.0;
    try {
        const auto q = imu.getQuaternion();
        target_heading_deg = bno055lib::fastExtractYawDeg(q.w, q.x, q.y, q.z);
    } catch (...) {
        target_heading_deg = 0.0;
    }

    // Configure Production HeadingController
    bno055lib::HeadingController::Config cfg;
    cfg.kp = 0.03;
    cfg.ki = 0.002;
    cfg.kd = 0.005;
    cfg.min_output = -0.5;
    cfg.max_output = 0.5;
    cfg.max_i_term = 0.1;

    bno055lib::HeadingController controller(cfg);
    constexpr double base_velocity = 0.5;

    char output_buffer[2048];
    char bar_corr[32], bar_left[32], bar_right[32];

    auto next_wake = std::chrono::steady_clock::now();

    while (true) {
        next_wake += LOOP_PERIOD;

        try {
            const auto quat = imu.getQuaternion();
            const auto gyro = imu.getGyroscope();
            const auto calib = imu.getCalibrationStatus();

            const double current_heading_deg = bno055lib::fastExtractYawDeg(quat.w, quat.x, quat.y, quat.z);
            const double gyro_z_deg = gyro.z * bno055lib::RAD_TO_DEG;

            // Compute PID output using production HeadingController
            auto out = controller.update(target_heading_deg, current_heading_deg, 0.01, gyro_z_deg, base_velocity);

            // Zero-allocation visual bar generation
            renderBarToBuffer(bar_corr, sizeof(bar_corr), out.correction * 2.0);
            renderBarToBuffer(bar_left, sizeof(bar_left), out.left_motor);
            renderBarToBuffer(bar_right, sizeof(bar_right), out.right_motor);

            // Fast single-syscall terminal update
            int len = std::snprintf(
                output_buffer, sizeof(output_buffer),
                "\033[H"
                "================ PRODUCTION BNO055 HEADING PID APP (100Hz) ================\n"
                "Calib: SYS=%d G=%d A=%d M=%d | Loop: 10ms (Zero-Alloc, Absolute Sleep)\n"
                "----------------------------------------------------------------------------\n"
                "Target Heading : %7.2f deg\n"
                "Current Heading: %7.2f deg\n"
                "Heading Error  : %7.2f deg\n"
                "Gyro Yaw Rate  : %7.2f deg/s\n"
                "----------------------------------------------------------------------------\n"
                "PID Correction (u): [%s] (%+6.3f)\n"
                "Left  Wheel Speed : [%s] (%5.1f%%)\n"
                "Right Wheel Speed : [%s] (%5.1f%%)\n"
                "----------------------------------------------------------------------------\n"
                "Press Ctrl+C to exit.\n",
                calib.sys, calib.gyro, calib.accel, calib.mag,
                target_heading_deg, current_heading_deg, out.error_deg, gyro_z_deg,
                bar_corr, out.correction,
                bar_left, out.left_motor * 100.0,
                bar_right, out.right_motor * 100.0
            );

            if (BNO055_LIKELY(len > 0)) {
                std::cout.write(output_buffer, len);
                std::cout.flush();
            }

        } catch (const bno055lib::IMUError& e) {
            std::cerr << "Communication error: " << e.what() << "\n";
        }

        std::this_thread::sleep_until(next_wake);
    }

    return 0;
}
