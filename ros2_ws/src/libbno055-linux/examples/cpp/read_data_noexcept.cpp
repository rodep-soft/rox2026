#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

#include "libbno055-linux/bno055.hpp"

// Flag to coordinate clean shutdown on Ctrl+C
std::atomic<bool> keep_running{true};

void signalHandler(int signum) {
    (void)signum;
    keep_running = false;
}

int main(int argc, char* argv[]) {
    std::string device = "/dev/i2c-1";
    if (argc > 1) {
        device = argv[1];
    }

    // Register signal handler for clean exit
    std::signal(SIGINT, signalHandler);

    std::cout << "Initializing BNO055 IMU on " << device << "..." << "\n";
    bno055lib::BNO055 imu(static_cast<uint8_t>(0x28), device);

    // Set custom logger to capture internal connection warnings/errors
    imu.setLogger([](bno055lib::LogLevel level, std::string_view message) {
        std::string label;
        switch (level) {
            case bno055lib::LogLevel::Debug:
                label = "[DEBUG]";
                break;
            case bno055lib::LogLevel::Info:
                label = "[INFO]";
                break;
            case bno055lib::LogLevel::Warning:
                label = "[WARN]";
                break;
            case bno055lib::LogLevel::Error:
                label = "[ERR]";
                break;
        }
        std::cerr << label << " " << message << "\n";
    });

    // Start in NDOF (9-DoF sensor fusion) mode
    if (!imu.begin(bno055lib::OpMode::NDOF)) {
        std::cerr << "Initialization failed! Check hardware connections or I2C permissions." << "\n";
        return 1;
    }

    // Optional: Use external crystal for better accuracy
    imu.setExtCrystalUse(true);

    std::cout << "IMU initialized. Reading data at 20Hz (Ctrl+C to exit)..." << "\n";
    std::cout << std::fixed << std::setprecision(3);

    auto next_loop = std::chrono::steady_clock::now();
    uint32_t loop_count = 0;

    while (keep_running) {
        next_loop += std::chrono::milliseconds(static_cast<int>(50));  // 20Hz

        // Read orientation (Quaternion) and angular velocity (Gyro) using noexcept APIs
        auto quat = imu.getQuaternionNoexcept();
        auto gyro = imu.getGyroscopeNoexcept();
        auto accel = imu.getLinearAccelerationNoexcept();

        if (quat && gyro && accel) {
            std::cout << "\r[Data] Quat(w,x,y,z): (" << quat->w << ", " << quat->x << ", " << quat->y << ", " << quat->z
                      << ") | Gyro(rad/s): (" << gyro->x << ", " << gyro->y << ", " << gyro->z << ") | Accel(m/s^2): ("
                      << accel->x << ", " << accel->y << ", " << accel->z << ")" << std::flush;
        } else {
            std::cout << "\r[Data] Reading failed (temporary I2C bus error)                 " << std::flush;
        }

        // Periodically print diagnostics telemetry every 2 seconds (40 loops)
        if (++loop_count % 40 == 0) {
            auto diag = imu.getDiagnostics();
            auto calib = imu.getCalibrationStatus();
            std::cout << "\n[DIAG] Calib: SYS=" << (int)calib.sys << " G=" << (int)calib.gyro
                      << " A=" << (int)calib.accel << " M=" << (int)calib.mag
                      << " | Bus Stats: RxErr=" << diag.read_failures << " TxErr=" << diag.write_failures
                      << " Reconnects=" << diag.reconnect_attempts << "\n";
        }

        std::this_thread::sleep_until(next_loop);
    }

    std::cout << "\nShutting down IMU..." << "\n";
    imu.enterSuspendMode();  // Power down the sensor to save energy
    std::cout << "IMU suspended. Exited cleanly." << "\n";

    return 0;
}
