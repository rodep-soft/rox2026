#include <chrono>
#include <iostream>
#include <thread>

#include "libbno055-linux/bno055.hpp"

int main(int argc, char* argv[]) {
    std::string device = "/dev/i2c-1";
    std::string output_file = "bno055_calib.bin";

    if (argc > 1) {
        device = argv[1];
    }
    if (argc > 2) {
        output_file = argv[2];
    }

    std::cout << "Initializing BNO055 on " << device << "..." << "\n";
    bno055lib::BNO055 imu(static_cast<uint8_t>(0x28), device);

    // Setup logger callback
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
        std::cout << label << " " << message << "\n";
    });

    if (!imu.begin(bno055lib::OpMode::NDOF)) {
        std::cerr << "Failed to initialize BNO055!" << "\n";
        return 1;
    }

    std::cout << "\n--- BNO055 Calibration Utility ---" << "\n";
    std::cout << "Please move the sensor to calibrate it:" << "\n";
    std::cout << "  - Gyroscope: Keep the sensor completely still for a few seconds." << "\n";
    std::cout << "  - Magnetometer: Move the sensor in a figure-8 pattern through the air." << "\n";
    std::cout << "  - Accelerometer: Place the sensor in 6 different stable positions." << "\n";
    std::cout << "----------------------------------\n" << "\n";

    while (true) {
        auto status = imu.getCalibrationStatus();
        auto diag = imu.getDiagnostics();

        std::cout << "\rCalib: SYS=" << static_cast<int>(status.sys) << " GYRO=" << static_cast<int>(status.gyro)
                  << " ACCEL=" << static_cast<int>(status.accel) << " MAG=" << static_cast<int>(status.mag)
                  << " | Diagnostics: RxErr=" << diag.read_failures << " TxErr=" << diag.write_failures
                  << " Reconn=" << diag.reconnect_attempts << "   " << std::flush;

        if (status.isFullyCalibrated()) {
            std::cout << "\n\nSensor is fully calibrated!" << "\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(200)));
    }

    std::cout << "Saving calibration to " << output_file << "..." << "\n";
    if (imu.saveCalibrationFile(output_file)) {
        std::cout << "Calibration saved successfully!" << "\n";
    } else {
        std::cerr << "Failed to save calibration file." << "\n";
        return 1;
    }

    return 0;
}
