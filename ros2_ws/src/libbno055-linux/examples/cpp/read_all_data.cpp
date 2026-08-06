#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "libbno055-linux/bno055.hpp"

constexpr uint8_t DEFAULT_ADDR = 0x28;
constexpr int OUTPUT_WIDTH = 7;
constexpr int LOOP_DELAY_MS = 100;

// Helper function to print a simple ASCII visual bar indicator for orientation angle [-90, 90]
void printBarIndicator(const std::string& label, double val_deg) {
    const int width = 20;
    const int center = width / 2;

    // Clamp to range
    if (val_deg < -90.0) val_deg = -90.0;
    if (val_deg > 90.0) val_deg = 90.0;

    int pos = center + static_cast<int>((val_deg / 90.0) * center);
    if (pos < 0) pos = 0;
    if (pos >= width) pos = width - 1;

    std::string bar(width, ' ');
    bar[center] = '|';
    bar[pos] = '*';

    std::cout << label << " [" << bar << "] " << std::fixed << std::setprecision(2) << std::setw(OUTPUT_WIDTH) << val_deg << " deg"
              << "\n";
}

int main(int argc, char* argv[]) {
    std::string device = "/dev/i2c-1";
    bno055lib::OpMode mode = bno055lib::OpMode::NDOF;
    std::string mode_str = "ndof";

    if (argc > 1) {
        device = argv[1];
    }
    if (argc > 2) {
        mode_str = argv[2];
        if (mode_str == "imu") {
            mode = bno055lib::OpMode::IMUPlus;
        } else if (mode_str == "amg") {
            mode = bno055lib::OpMode::AMG;
        } else if (mode_str == "gyro") {
            mode = bno055lib::OpMode::GyroOnly;
        } else if (mode_str == "ndof") {
            mode = bno055lib::OpMode::NDOF;
        } else {
            std::cerr << "Unknown mode: " << mode_str << ". Defaulting to ndof." << "\n";
            mode_str = "ndof";
        }
    }

    std::cout << "Initializing BNO055 IMU on " << device << " (Mode: " << mode_str << ")..." << "\n";
    bno055lib::BNO055 imu(DEFAULT_ADDR, device);

    // Use default logger (standard error output)
    if (!imu.begin(mode)) {
        std::cerr << "Failed to initialize BNO055!" << "\n";
        return 1;
    }

    std::cout << "BNO055 initialized. Displaying all sensor readings (10Hz)..." << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));

    while (true) {
        try {
            // Read all physical data types provided by BNO055
            auto accel = imu.getAccelerometer();
            auto mag = imu.getMagnetometer();
            auto gyro = imu.getGyroscope();
            auto euler = imu.getEulerAngles();
            auto linear = imu.getLinearAcceleration();
            auto gravity = imu.getGravity();
            auto quat = imu.getQuaternion();
            auto temp = imu.getTemperature();
            auto calib = imu.getCalibrationStatus();

            // Convert quaternion to euler degrees for human visualization
            auto euler_deg = bno055lib::toEulerDegrees(quat);

            // Clear console using ANSI escape codes for clean terminal refresh
            std::cout << "\033[2J\033[H";
            std::cout << "================== BNO055 Visual Dashboard ==================" << "\n";
            std::cout << "Device: " << device << "  |  Mode: " << mode_str << "\n";
            std::cout << "Calibration: SYS=" << static_cast<int>(calib.sys) << " GYRO=" << static_cast<int>(calib.gyro)
                      << " ACCEL=" << static_cast<int>(calib.accel) << " MAG=" << static_cast<int>(calib.mag) << "\n";
            std::cout << "-------------------------------------------------------------" << "\n";

            // Visual indicators for tilt (Roll and Pitch)
            printBarIndicator("Roll  (X)", euler_deg.x);
            printBarIndicator("Pitch (Y)", euler_deg.y);
            std::cout << "Yaw   (Z)  " << std::fixed << std::setprecision(2) << std::setw(OUTPUT_WIDTH) << euler_deg.z << " deg"
                      << "\n";
            std::cout << "-------------------------------------------------------------" << "\n";

            std::cout << "Accelerometer (m/s^2):       X=" << std::setw(OUTPUT_WIDTH) << accel.x << " Y=" << std::setw(OUTPUT_WIDTH)
                      << accel.y << " Z=" << std::setw(OUTPUT_WIDTH) << accel.z << "\n";
            std::cout << "Magnetometer (uT):           X=" << std::setw(OUTPUT_WIDTH) << mag.x << " Y=" << std::setw(OUTPUT_WIDTH) << mag.y
                      << " Z=" << std::setw(OUTPUT_WIDTH) << mag.z << "\n";
            std::cout << "Gyroscope (rad/s):           X=" << std::setw(OUTPUT_WIDTH) << gyro.x << " Y=" << std::setw(OUTPUT_WIDTH) << gyro.y
                      << " Z=" << std::setw(OUTPUT_WIDTH) << gyro.z << "\n";
            std::cout << "Euler Angles (rad):          Roll=" << std::setw(OUTPUT_WIDTH) << euler.x << " Pitch=" << std::setw(OUTPUT_WIDTH)
                      << euler.y << " Yaw=" << std::setw(OUTPUT_WIDTH) << euler.z << "\n";
            std::cout << "Linear Acceleration (m/s^2):  X=" << std::setw(OUTPUT_WIDTH) << linear.x << " Y=" << std::setw(OUTPUT_WIDTH)
                      << linear.y << " Z=" << std::setw(OUTPUT_WIDTH) << linear.z << "\n";
            std::cout << "Gravity Vector (m/s^2):       X=" << std::setw(OUTPUT_WIDTH) << gravity.x << " Y=" << std::setw(OUTPUT_WIDTH)
                      << gravity.y << " Z=" << std::setw(OUTPUT_WIDTH) << gravity.z << "\n";
            std::cout << "Quaternion:                  W=" << std::setw(OUTPUT_WIDTH) << quat.w << " X=" << std::setw(OUTPUT_WIDTH) << quat.x
                      << " Y=" << std::setw(OUTPUT_WIDTH) << quat.y << " Z=" << std::setw(OUTPUT_WIDTH) << quat.z << "\n";
            std::cout << "Temperature (C):             " << static_cast<int>(temp) << "\n";

        } catch (const bno055lib::IMUError& e) {
            std::cout << "\033[2J\033[H";
            std::cerr << "Communication error: " << e.what() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_DELAY_MS));  // 10Hz
    }

    return 0;
}
