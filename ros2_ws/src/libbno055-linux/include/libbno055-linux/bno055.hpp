#ifndef LIBBNO055_LINUX_BNO055_HPP
#define LIBBNO055_LINUX_BNO055_HPP

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include "libbno055-linux/transport.hpp"

#ifndef BNO055_LIKELY
#if defined(__GNUC__) || defined(__clang__)
#define BNO055_LIKELY(x)   __builtin_expect(!!(x), 1)
#define BNO055_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define BNO055_LIKELY(x)   (x)
#define BNO055_UNLIKELY(x) (x)
#endif
#endif

namespace bno055lib {

// Log levels for customization
enum class LogLevel : uint8_t { Debug, Info, Warning, Error };

// Logger callback type
using LoggerCallback = std::function<void(LogLevel level, std::string_view message)>;

// Custom exception class for IMU errors
class IMUError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Standard 3D vector for physical readings
struct Vector3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    float operator[](size_t index) const {
        switch (index) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                throw std::out_of_range("Vector3 index out of range");
        }
    }

    float& operator[](size_t index) {
        switch (index) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                throw std::out_of_range("Vector3 index out of range");
        }
    }
};

// Standard Quaternion for rotation representation
struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

// Calibration offsets
struct Offsets {
    int16_t accel_offset_x{0};
    int16_t accel_offset_y{0};
    int16_t accel_offset_z{0};

    int16_t mag_offset_x{0};
    int16_t mag_offset_y{0};
    int16_t mag_offset_z{0};

    int16_t gyro_offset_x{0};
    int16_t gyro_offset_y{0};
    int16_t gyro_offset_z{0};

    int16_t accel_radius{0};
    int16_t mag_radius{0};
};

// Calibration status for various sensors
struct CalibrationStatus {
    uint8_t sys{0};
    uint8_t gyro{0};
    uint8_t accel{0};
    uint8_t mag{0};

    bool isFullyCalibrated() const { return sys == 3 && gyro == 3 && accel == 3 && mag == 3; }
};

// Diagnostics information for telemetry and hardware monitoring
struct Diagnostics {
    uint32_t write_failures{0};
    uint32_t read_failures{0};
    uint32_t reconnect_attempts{0};
};

// BNO055 Operation Modes
enum class OpMode : uint8_t {
    Config = 0X00,
    AccOnly = 0X01,
    MagOnly = 0X02,
    GyroOnly = 0X03,
    AccMag = 0X04,
    AccGyro = 0X05,
    MagGyro = 0X06,
    AMG = 0X07,
    IMUPlus = 0X08,
    Compass = 0X09,
    M4G = 0X0A,
    NDOF_FMC_Off = 0X0B,
    NDOF = 0X0C
};

// Axis Remap Configurations
enum class AxisMapConfig : uint8_t {
    P0 = 0x21,
    P1 = 0x24,  // Default
    P2 = 0x24,
    P3 = 0x21,
    P4 = 0x24,
    P5 = 0x21,
    P6 = 0x21,
    P7 = 0x24
};

enum class AxisMapSign : uint8_t {
    P0 = 0x04,
    P1 = 0x00,  // Default
    P2 = 0x06,
    P3 = 0x02,
    P4 = 0x03,
    P5 = 0x01,
    P6 = 0x07,
    P7 = 0x05
};

class BNO055 {
public:
    // UART Configuration
    struct UARTConfig {
        std::string port{"/dev/ttyUSB0"};
        uint32_t baudrate{115200};
        double timeout{0.1};
        bool low_latency{false};
    };

    // Constructors
    // I2C mode: Default BNO055_ADDRESS_A is 0x28. Alternate is 0x29.
    // i2c_device: Linux I2C device node.
    explicit BNO055(uint8_t i2c_address = 0x28, std::string_view i2c_device = "/dev/i2c-1");

    // UART mode: Connect using a USB-to-UART bridge
    explicit BNO055(const UARTConfig& uart_config);

    // Custom transport mode for testing / dependency injection
    explicit BNO055(std::unique_ptr<Transport> transport);

    // Destructor
    ~BNO055();

    // Disable copy semantics, enable move semantics (RAII for fd)
    BNO055(const BNO055&) = delete;
    BNO055& operator=(const BNO055&) = delete;
    BNO055(BNO055&&) noexcept;
    BNO055& operator=(BNO055&&) noexcept;

    // Initialization
    // Initialize the IMU and set the operating mode
    bool begin(OpMode mode = OpMode::NDOF);

    // Hardware reset the IMU and restore the current operating mode
    bool reset();

    // Set / Get operation mode
    void setMode(OpMode mode);
    OpMode getMode();

    // Configure axis map config and sign
    void setAxisRemap(AxisMapConfig config);
    void setAxisSign(AxisMapSign sign);

    // Set whether to use external 32.768kHz crystal
    void setExtCrystalUse(bool use_xtal);

    // Data getters (converted to standard SI units: m/s^2, rad/s, rad, uT)
    Vector3 getAccelerometer();
    Vector3 getMagnetometer();
    Vector3 getGyroscope();
    Vector3 getEulerAngles();
    Vector3 getLinearAcceleration();
    Vector3 getGravity();
    Quaternion getQuaternion();
    int8_t getTemperature();

    // Non-throwing data getters (returns std::nullopt on failure instead of throwing IMUError)
    std::optional<Vector3> getAccelerometerNoexcept() noexcept;
    std::optional<Vector3> getMagnetometerNoexcept() noexcept;
    std::optional<Vector3> getGyroscopeNoexcept() noexcept;
    std::optional<Vector3> getEulerAnglesNoexcept() noexcept;
    std::optional<Vector3> getLinearAccelerationNoexcept() noexcept;
    std::optional<Vector3> getGravityNoexcept() noexcept;
    std::optional<Quaternion> getQuaternionNoexcept() noexcept;
    std::optional<int8_t> getTemperatureNoexcept() noexcept;

    // EKF Burst Reading APIs
    struct RawSensorData {
        Vector3 accel;
        Vector3 mag;
        Vector3 gyro;
    };
    /// Retrieve all raw sensor outputs (accelerometer, magnetometer, gyroscope) in a single burst read transaction.
    /// Designed to reduce I2C transaction latency for state estimation filters (EKF).
    RawSensorData getRawSensorData();
    std::optional<RawSensorData> getRawSensorDataNoexcept() noexcept;

    // Beginner-friendly data getters (returns zero/default values on failure, no exceptions, no optionals)
    Vector3 getAccelerometerOrDefault() noexcept;
    Vector3 getMagnetometerOrDefault() noexcept;
    Vector3 getGyroscopeOrDefault() noexcept;
    Vector3 getEulerAnglesOrDefault() noexcept;
    Vector3 getLinearAccelerationOrDefault() noexcept;
    Vector3 getGravityOrDefault() noexcept;
    Quaternion getQuaternionOrDefault() noexcept;
    int8_t getTemperatureOrDefault() noexcept;

    // Diagnostics getter
    Diagnostics getDiagnostics() const noexcept;

    // Calibration and Offset configuration
    CalibrationStatus getCalibrationStatus();
    bool getSensorOffsets(Offsets& offsets);
    bool getSensorOffsets(std::array<uint8_t, 22>& calib_data);
    void setSensorOffsets(const Offsets& offsets);
    void setSensorOffsets(const std::array<uint8_t, 22>& calib_data);

    // Calibration loading and saving helper functions (writes 22 bytes binary file)
    bool saveCalibrationFile(std::string_view filepath);
    bool loadCalibrationFile(std::string_view filepath);

    // Power modes
    void enterSuspendMode();
    void enterNormalMode();

    // Custom Logger hook
    void setLogger(LoggerCallback callback);

    // Asynchronous & Callback-driven Reading API
    struct AllData {
        Vector3 accel;
        Vector3 mag;
        Vector3 gyro;
        Vector3 euler;
        Vector3 linear_accel;
        Vector3 gravity;
        Quaternion quat;
        int8_t temp;
    };
    using AsyncDataCallback = std::function<void(const AllData& data)>;

    /// Start a background thread to poll data asynchronously.
    /// @param rate_hz The polling rate in Hz (e.g. 50.0).
    /// @param callback Callback executed every time new data is retrieved.
    bool startAsyncReading(double rate_hz, AsyncDataCallback callback);

    /// Stop the background async reading thread.
    void stopAsyncReading();

    /// Burst-read ALL sensor outputs in a single 45-byte I2C transaction (registers 0x08–0x34).
    /// Returns accel, mag, gyro, euler, quaternion, linear_accel, gravity, and temperature atomically.
    /// 8× fewer I2C transactions compared to reading sensors individually.
    std::optional<AllData> getAllDataNoexcept() noexcept;

    // Asynchronous Raw Reading API
    using RawAsyncDataCallback = std::function<void(const RawSensorData& data)>;
    bool startRawAsyncReading(double rate_hz, RawAsyncDataCallback callback);
    void stopRawAsyncReading();

    // Linux GPIO Hardware Interrupt (IRQ) Driven API
    /// Start a background thread waiting for a GPIO hardware interrupt (INT pin rising edge).
    /// Bypasses polling delays entirely.
    /// @param gpio_pin Linux GPIO pin number connected to BNO055 INT pin (e.g. 24).
    /// @param callback Callback executed the exact microsecond the hardware interrupt fires.
    bool startInterruptDrivenReading(int gpio_pin, RawAsyncDataCallback callback);
    void stopInterruptDrivenReading();

    // Automatic Calibration load/save configuration
    /// Configure automatic calibration files. If configured, calibration will be
    /// loaded on begin(), and dynamically saved when the calibration status reaches maximum.
    void enableAutoCalibration(std::string_view filepath);
    void disableAutoCalibration();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// Utility function to convert Quaternion representation to Euler Angles (Roll, Pitch, Yaw) in degrees.
// Mapping: x = Roll (-180 to 180), y = Pitch (-90 to 90), z = Yaw (0 to 360).
Vector3 toEulerDegrees(const Quaternion& q) noexcept;

}  // namespace bno055lib

#endif  // LIBBNO055_LINUX_BNO055_HPP
