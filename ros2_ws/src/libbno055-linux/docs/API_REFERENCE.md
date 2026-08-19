# libbno055-linux API Reference

> **Note on C++ Standard & Compatibility**:
> - **C++ API (`bno055.hpp`)**: Requires **C++17** or newer (`std::optional`, `std::string_view`).
> - **Legacy / Older Environments (C99, C++11, C++14)**: Use the C API (`bno055_c.h`) which wraps the C++ implementation behind a C99 FFI handle (`bno055_handle_t`).

<details>
<summary><strong>Table of Contents</strong></summary>

- [Namespaces & Types](#namespaces--types)
- [Class BNO055](#class-bno055)
  - [Lifecycle](#lifecycle)
  - [Configuration](#configuration)
  - [Sensor Data (Throwing APIs)](#sensor-data-throwing-apis)
  - [Sensor Data (Exception-free / noexcept APIs)](#sensor-data-exception-free--noexcept-apis)
  - [Diagnostics & Calibration](#diagnostics--calibration)
  - [Power Management](#power-management)
  - [Logging](#logging)
- [Utilities (Class-External)](#utilities-class-external)
- [Class HeadingController](#class-headingcontroller)

</details>

## Namespaces & Types

```cpp
namespace bno055lib {
    // 3D coordinate vector (used for accelerometer, gyroscope, magnetometer, euler, gravity, linear accel)
    struct Vector3 {
        float x;
        float y;
        float z;
    };

    // 3D orientation quaternion representation
    struct Quaternion {
        float w; // Real part
        float x; // Imaginary X
        float y; // Imaginary Y
        float z; // Imaginary Z
    };

    // Transport interface representing I/O connection to the BNO055 hardware
    class Transport {
    public:
        virtual ~Transport() = default;
        virtual bool open() = 0;
        virtual void close() = 0;
        virtual bool isOpen() const = 0;
        virtual bool write8(uint8_t reg, uint8_t value) = 0;
        virtual bool writeLen(uint8_t reg, const uint8_t* buffer, uint8_t len) = 0;
        virtual bool read8(uint8_t reg, uint8_t& value) = 0;
        virtual bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len) = 0;
    };

    // Mock implementation of Transport for hardware-independent verification
    class MockTransport : public Transport {
    public:
        MockTransport();
        void setRegister(uint8_t reg, uint8_t value);
        void setRegister16LE(uint8_t reg, int16_t value);
        void setFailOpen(bool fail);
        void setFailReads(bool fail);
        void setFailWrites(bool fail);
        uint32_t getWriteCount() const;
        uint32_t getReadCount() const;
        void reset();
    };

    // Bundled structure representing all sensor physical data
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
    
    // Binary calibration offsets (22 bytes total) for saving/restoring sensor profile
    struct Offsets {
        int16_t accel_offset_x, accel_offset_y, accel_offset_z;
        int16_t mag_offset_x, mag_offset_y, mag_offset_z;
        int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
        int16_t accel_radius, mag_radius;
    };

    // Dynamic calibration status of the sensor (0 = uncalibrated, 3 = fully calibrated)
    struct CalibrationStatus {
        uint8_t sys;   // Overall system calibration status [0-3]
        uint8_t gyro;  // Gyroscope calibration status [0-3]
        uint8_t accel; // Accelerometer calibration status [0-3]
        uint8_t mag;   // Magnetometer calibration status [0-3]
        
        // Returns true if gyro, accel, and mag are fully calibrated (status == 3)
        bool isFullyCalibrated() const;
    };

    // Cumulative telemetry diagnostics tracking error rates for health monitoring
    struct Diagnostics {
        uint32_t write_failures;      // Number of failed register write transactions
        uint32_t read_failures;       // Number of failed register read transactions
        uint32_t reconnect_attempts;  // Number of I2C bus auto-reconnect triggers
    };

    // Operating mode configuration
    enum class OpMode : uint8_t {
        Config = 0X00,          // Configuration mode (required to write map/sign/crystal settings)
        AccOnly = 0X01,         // Non-fusion Accelerometer only
        MagOnly = 0X02,         // Non-fusion Magnetometer only
        GyroOnly = 0X03,        // Non-fusion Gyroscope only
        AccMag = 0X04,          // Non-fusion Accelerometer + Magnetometer
        AccGyro = 0X05,         // Non-fusion Accelerometer + Gyroscope
        MagGyro = 0X06,         // Non-fusion Magnetometer + Gyroscope
        AMG = 0X07,             // Non-fusion Accelerometer + Magnetometer + Gyroscope (raw outputs)
        IMUPlus = 0X08,         // 6-axis Fusion (Acc + Gyro). Yaw relative to boot position. Recommended indoors.
        Compass = 0X09,         // 6-axis Fusion (Acc + Mag). Absolute Yaw.
        M4G = 0X0A,             // 6-axis Fusion (Mag + Gyro).
        NDOF_FMC_Off = 0X0B,    // 9-axis Fusion (Acc + Mag + Gyro) with Fast Magnetometer Calibration disabled
        NDOF = 0X0C             // 9-axis Fusion (Acc + Mag + Gyro) with FMC enabled. Absolute Yaw (North-referenced).
    };

    enum class LogLevel { Debug, Info, Warning, Error };
    using LoggerCallback = std::function<void(LogLevel level, std::string_view message)>;
}
```
### Class BNO055

All functions in the `BNO055` class are thread-safe and protect access to the underlying I2C bus using internal mutexes.

#### Lifecycle

*   **explicit BNO055(uint8_t i2c_address = 0x28, std::string_view i2c_device = "/dev/i2c-1")**
    *   *Parameters*:
        *   `i2c_address`: `uint8_t`. The I2C slave address of the BNO055 (typically `0x28` or `0x29`).
        *   `i2c_device`: `std::string_view`. The Linux file path to the I2C adapter (e.g., `/dev/i2c-1`).
    *   *Returns*: None (Constructor).
    *   *Description*: Creates a BNO055 handler. If compiled on macOS/Windows, it falls back to the mock mode and accepts any mock device name.
*   **explicit BNO055(std::unique_ptr<Transport> transport)**
    *   *Parameters*:
        *   `transport`: `std::unique_ptr<Transport>`. Custom transport implementation (e.g., a `MockTransport` instance).
    *   *Returns*: None (Constructor).
    *   *Description*: Creates a BNO055 handler using a custom transport layer. Extremely useful for unit testing (dependency injection) and porting.
*   **bool begin(OpMode mode = OpMode::NDOF)**
    *   *Parameters*:
        *   `mode`: `OpMode`. The target operation mode to start in.
    *   *Returns*: `bool`. `true` if initialization, self-test verification, and mode transition succeeded; `false` on boot timeouts or I2C failures.
    *   *Description*: Power-cycles the sensor, verifies the Chip ID, configures default Axis Maps and Unit settings, and spawns the background auto-recovery thread.
*   **bool reset()**
    *   *Parameters*: None.
    *   *Returns*: `bool`. `true` if reset and mode restoration succeeded; `false` on failure.
    *   *Description*: Triggers a software hardware-reset using the `SYS_TRIGGER` register and completely re-initializes the device into the previously active operating mode. Useful for recovery from external hardware lockups.

#### Configuration

*   **void setMode(OpMode mode)**
    *   *Parameters*:
        *   `mode`: `OpMode`. The target operation mode to switch into.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on permanent I2C communication failures.
    *   *Description*: Switches the operating mode of the sensor. Handles transition delay requirements automatically.
*   **OpMode getMode()**
    *   *Parameters*: None.
    *   *Returns*: `OpMode`. The active operation mode currently registered on the sensor.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setAxisRemap(AxisMapConfig config)**
    *   *Parameters*:
        *   `config`: `AxisMapConfig`. The remap configuration matching the sensor's mounting orientation.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setAxisSign(AxisMapSign sign)**
    *   *Parameters*:
        *   `sign`: `AxisMapSign`. The axis sign configuration to adjust rotation/acceleration directions.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setExtCrystalUse(bool use_xtal)**
    *   *Parameters*:
        *   `use_xtal`: `bool`. Set to `true` to use the external 32.768kHz crystal oscillator for enhanced fusion stability.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.

#### Sensor Data (Throwing APIs)

These functions query raw data registers and convert them to SI units. They throw `bno055lib::IMUError` on permanent I2C communication loss.

*   **Vector3 getAccelerometer()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis acceleration in meters per second squared (m/s^2).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getMagnetometer()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis magnetic field strength in microteslas (uT).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getGyroscope()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis angular velocity in radians per second (rad/s).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getEulerAngles()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. Roll, Pitch, Yaw in radians (rad). Mapping: `x` = Roll, `y` = Pitch, `z` = Yaw.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getLinearAcceleration()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis linear acceleration (accelerometer excluding gravity) in m/s^2.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getGravity()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis gravity vector in m/s^2.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Quaternion getQuaternion()**
    *   *Parameters*: None.
    *   *Returns*: `Quaternion`. Normalized 3D orientation unit quaternion (w, x, y, z).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **int8_t getTemperature()**
    *   *Parameters*: None.
    *   *Returns*: `int8_t`. Chip temperature in degrees Celsius (C).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **RawSensorData getRawSensorData()**
    *   *Parameters*: None.
    *   *Returns*: `RawSensorData`. Struct containing converted raw accelerometer, magnetometer, and gyroscope vectors.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
    *   *Description*: Sequentially reads 18 bytes of raw sensor data in a single burst I2C transaction, minimizing bus occupancy.

#### Sensor Data (Exception-free / noexcept APIs)

These companion APIs perform the exact same register queries and conversions but never throw exceptions.

*   **std::optional\<Vector3\> getAccelerometerNoexcept() noexcept**
*   **std::optional\<Vector3\> getMagnetometerNoexcept() noexcept**
*   **std::optional\<Vector3\> getGyroscopeNoexcept() noexcept**
*   **std::optional\<Vector3\> getEulerAnglesNoexcept() noexcept**
*   **std::optional\<Vector3\> getLinearAccelerationNoexcept() noexcept**
*   **std::optional\<Vector3\> getGravityNoexcept() noexcept**
*   **std::optional\<Quaternion\> getQuaternionNoexcept() noexcept**
*   **std::optional\<int8_t\> getTemperatureNoexcept() noexcept**
*   **std::optional\<RawSensorData\> getRawSensorDataNoexcept() noexcept**
    *   *Parameters*: None.
    *   *Returns*: `std::optional<T>`. Contains the requested struct on success; `std::nullopt` on communication failure.
    *   *Description*: Safety-hardened read path that increments I2C diagnostic counters upon failure without generating CPU exceptions.
*   **std::optional\<AllData\> getAllDataNoexcept() noexcept**
    *   *Parameters*: None.
    *   *Returns*: `std::optional<AllData>`. Full snapshot of all 8 sensor outputs (accel, mag, gyro, euler, quaternion, linear accel, gravity, temp).
    *   *Description*: Executes a single 45-byte burst I2C transaction covering registers `0x08` through `0x34`. Reduces bus transactions by 87.5% compared to individual queries. Atomic capture across all channels.

    *   *Parameters*: None.
    *   *Returns*: The requested struct directly (`Vector3`, `Quaternion`, or `int8_t`). On temporary bus drops, returns the last cached valid frame (or zero/identity).

#### Diagnostics & Calibration

*   **Diagnostics getDiagnostics() const noexcept**
    *   *Parameters*: None.
    *   *Returns*: `Diagnostics`. The telemetry diagnostic struct containing I2C read/write error counts and reconnection attempts.
*   **CalibrationStatus getCalibrationStatus()**
    *   *Parameters*: None.
    *   *Returns*: `CalibrationStatus`. Current calibration state values (0 to 3) for the system, gyro, accelerometer, and magnetometer.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **bool getSensorOffsets(Offsets& offsets)**
    *   *Parameters*:
        *   `offsets`: `Offsets&` (output). Structure to store the retrieved calibration offsets.
    *   *Returns*: `bool`. `true` if offsets were read and stored successfully; `false` on I2C failure.
*   **bool getSensorOffsets(std::array<uint8_t, 22>& calib_data)**
    *   *Parameters*:
        *   `calib_data`: `std::array<uint8_t, 22>&` (output). Raw byte array to store the 22 bytes of offsets.
    *   *Returns*: `bool`. `true` if read succeeded; `false` on failure.
*   **void setSensorOffsets(const Offsets& offsets)**
    *   *Parameters*:
        *   `offsets`: `const Offsets&`. Calibration offset parameters to load into the sensor registers.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Writes offsets using a single-batch sequential I2C write transaction to minimize bus occupation.
*   **void setSensorOffsets(const std::array<uint8_t, 22>& calib_data)**
    *   *Parameters*:
        *   `calib_data`: `const std::array<uint8_t, 22>&`. Raw 22-byte array containing calibration offsets.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
*   **bool saveCalibrationFile(std::string_view filepath)**
    *   *Parameters*:
        *   `filepath`: `std::string_view`. Destination file path to save the 22-byte profile (e.g., `calib.bin`).
    *   *Returns*: `bool`. `true` if offsets were read and successfully written to disk; `false` on I2C or file I/O errors.
*   **bool loadCalibrationFile(std::string_view filepath)**
    *   *Parameters*:
        *   `filepath`: `std::string_view`. Source path of the 22-byte binary profile.
    *   *Returns*: `bool`. `true` if file read, register upload, and cache storage succeeded; `false` on file I/O or I2C errors.
    *   *Description*: Uploads calibration to the sensor and caches it locally so it can be automatically restored during an I2C hot-reconnect recovery.

#### Power Management

*   **void enterSuspendMode()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Places the sensor into suspend mode to reduce power consumption. Suspends accelerometer, gyroscope, and magnetometer blocks.
*   **void enterNormalMode()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Awakes the sensor from suspend mode back to active normal operation.

#### Logging

*   **void setLogger(LoggerCallback callback)**
    *   *Parameters*:
        *   `callback`: `LoggerCallback`. Callback function of signature `void(LogLevel level, std::string_view message)`.
    *   *Returns*: `void`.
    *   *Description*: Hooks a custom logging function (such as `std::cout` or ROS 2 logging macros) to direct library diagnostics, warnings, and reconnect traces.

#### Asynchronous Reading

*   **bool startAsyncReading(double rate_hz, AsyncDataCallback callback)**
    *   *Parameters*:
        *   `rate_hz`: `double`. The target frequency in Hz to query the sensor data.
        *   `callback`: `AsyncDataCallback`. Callback function executed on the background thread whenever new measurements are gathered.
    *   *Returns*: `bool`. `true` if background polling thread successfully spawned; `false` if already running.
    *   *Description*: Spawns a high-priority background polling thread that sleeps to minimize timing jitter. Calls `callback` with a thread-safe copy of `AllData`.
*   **void stopAsyncReading()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Description*: Signals the background async thread to terminate and blocks until it joins.
*   **bool startRawAsyncReading(double rate_hz, RawAsyncDataCallback callback)**
    *   *Parameters*:
        *   `rate_hz`: `double`. Polling frequency.
        *   `callback`: `RawAsyncDataCallback`. Callback executing with raw 3-axis readings.
    *   *Returns*: `bool`.
    *   *Description*: Spawns a lightweight async thread polling only raw IMU data via burst transaction.
*   **void stopRawAsyncReading()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Description*: Stops the raw async polling thread.

#### Interrupt Driven Reading (IRQ)

*   **bool startInterruptDrivenReading(int gpio_pin, RawAsyncDataCallback callback)**
    *   *Parameters*:
        *   `gpio_pin`: `int`. Linux GPIO pin connected to the BNO055 INT pin.
        *   `callback`: `RawAsyncDataCallback`. Handler triggered on interrupt.
    *   *Returns*: `bool`. `true` if background IRQ waiting thread spawned.
    *   *Description*: Registers a rising-edge trigger on the specified Linux GPIO pin. When BNO055 triggers INT, the kernel interrupt immediately unblocks POSIX `poll()` on the GPIO sysfs file descriptors, firing the callback with raw burst readings at sub-millisecond latency.
*   **void stopInterruptDrivenReading()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Description*: Stops the interrupt-driven IRQ monitoring thread.

#### Automatic Calibration

*   **void enableAutoCalibration(std::string_view filepath)**
    *   *Parameters*:
        *   `filepath`: `std::string_view`. The binary calibration profile path to load from on `begin()` and save to.
    *   *Returns*: `void`.
    *   *Description*: Activates automated calibration loading on startup and writes offsets back to the disk target once fully calibrated status is reached.
*   **void disableAutoCalibration()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Description*: Deactivates the automated calibration load/save helper logic.

### Utilities (Class-External)

*   **Vector3 toEulerDegrees(const Quaternion& q) noexcept**
    *   *Parameters*:
        *   `q`: `const Quaternion&`. The normalized unit quaternion representation of orientation.
    *   *Returns*: `Vector3`. Roll, Pitch, and Yaw in degrees (Mapping: `x` = Roll `[-180, 180]`, `y` = Pitch `[-90, 90]`, `z` = Yaw `[0, 360)`).
    *   *Description*: Utility function to convert Quaternion orientation into human-readable Euler angles in degrees (now executing on single-precision `float` elements).

---

## C API Reference (`libbno055-linux/bno055_c.h`)

The library includes a native C API (`extern "C"`) allowing seamless integration with C projects, Python (`ctypes`/`cffi`), Rust (`bindgen`), Go, and Zig.

```c
#include <libbno055-linux/bno055_c.h>

// 1. Create Handle
bno055_handle_t imu = bno055_create_i2c(0x28, "/dev/i2c-1");

// 2. Initialize
if (bno055_begin(imu, BNO055_OPMODE_NDOF)) {
    bno055_quaternion_t q;
    if (bno055_get_quaternion(imu, &q)) {
        bno055_vector3_t euler = bno055_to_euler_degrees(&q);
        printf("Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", euler.x, euler.y, euler.z);
    }
}

// 3. Destroy
bno055_destroy(imu);
```

---

## Python API Reference (`import libbno055`)

The Python module is built using `pybind11` and strictly mirrors the C++17 API, providing native C++ performance with Pythonic syntax. All hardware exceptions are caught at the C++ boundary, and failed reads gracefully return `None`.

### Classes & Types

*   **`libbno055.Vector3`**: Has properties `x`, `y`, `z` (floats).
*   **`libbno055.Quaternion`**: Has properties `w`, `x`, `y`, `z` (floats).
*   **`libbno055.CalibrationStatus`**: Has properties `sys`, `gyro`, `accel`, `mag` (integers 0-3), and method `is_fully_calibrated() -> bool`.
*   **`libbno055.Diagnostics`**: Has properties `write_failures`, `read_failures`, `reconnect_attempts` (integers).
*   **`libbno055.OpMode`**: Enum (e.g., `libbno055.OpMode.NDOF`, `libbno055.OpMode.IMUPlus`, `libbno055.OpMode.Config`).

### Class: `BNO055`

#### Constructors
*   **`__init__(self, address: int = 0x28, device: str = "/dev/i2c-1")`**
    Initializes the IMU for I2C communication.
*   **`__init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1)`**
    Initializes the IMU for USB-to-UART serial communication.

#### Configuration & Lifecycle
*   **`begin(self, mode: OpMode = OpMode.NDOF) -> bool`**
    Resets the sensor, verifies connection, and sets the target operation mode. Returns `True` on success.
*   **`reset(self) -> bool`**
    Triggers a soft hardware-reset and restores the current operation mode.
*   **`set_mode(self, mode: OpMode) -> None`**
    Switches the operating mode dynamically.

#### Sensor Data Reading
*These methods return the corresponding data structure on success, or `None` if an I2C/UART communication failure occurs.*

*   **`get_accelerometer(self) -> Vector3 | None`**: Returns acceleration (m/s^2).
*   **`get_magnetometer(self) -> Vector3 | None`**: Returns magnetic field (uT).
*   **`get_gyroscope(self) -> Vector3 | None`**: Returns angular velocity (rad/s).
*   **`get_euler_angles(self) -> Vector3 | None`**: Returns Roll, Pitch, Yaw (rad).
*   **`get_linear_acceleration(self) -> Vector3 | None`**: Returns linear accel (m/s^2).
*   **`get_gravity(self) -> Vector3 | None`**: Returns gravity vector (m/s^2).
*   **`get_quaternion(self) -> Quaternion | None`**: Returns unit quaternion.
*   **`get_temperature(self) -> int | None`**: Returns chip temperature (°C).

#### Diagnostics & Utilities
*   **`get_calibration_status(self) -> CalibrationStatus | None`**
    Fetches the live calibration status (0 to 3) for all sensor blocks.
*   **`get_diagnostics(self) -> Diagnostics`**
    Returns internal telemetry tracking auto-reconnects and dropped frames.
*   **`libbno055.to_euler_degrees(q: Quaternion) -> Vector3`**
    (Module-level function). Converts a quaternion into Euler angles in degrees (`x`=Roll, `y`=Pitch, `z`=Yaw).

---

## Rust API Reference (`use libbno055::*`)

The Rust crate (`libbno055`) is published on [crates.io](https://crates.io/crates/libbno055) and provides safe, idiomatic Rust wrappers around the C++ engine using Foreign Function Interfaces (FFI). 

### Installation
```bash
cargo add libbno055
```

### Types & Structs
All structures implement `Clone`, `Copy`, and `Debug`.

*   **`pub struct Vector3 { pub x: f32, pub y: f32, pub z: f32 }`**
*   **`pub struct Quaternion { pub w: f32, pub x: f32, pub y: f32, pub z: f32 }`**
*   **`pub struct CalibrationStatus { pub sys: u8, pub gyro: u8, pub accel: u8, pub mag: u8 }`**
    *   `impl CalibrationStatus { pub fn is_fully_calibrated(&self) -> bool }`
*   **`pub struct Diagnostics { pub write_failures: u32, pub read_failures: u32, pub reconnect_attempts: u32 }`**
*   **`pub struct RawSensorData { pub accel: Vector3, pub mag: Vector3, pub gyro: Vector3 }`**
*   **`pub enum OpMode`**: Includes variants like `Config`, `AccOnly`, `IMUPlus`, `NDOF`, etc.

### Struct: `BNO055`

#### Constructors
*   **`pub fn new_i2c(address: u8, device: &str) -> Result<BNO055, &'static str>`**
    Attempts to create a BNO055 handler targeting an I2C device node. Fails if the C++ handler cannot be allocated.
*   **`pub fn new_uart(port: &str, baudrate: u32) -> Result<BNO055, &'static str>`**
    Attempts to create a BNO055 handler targeting a Serial UART port.

#### Lifecycle & Configuration
*   **`pub fn begin(&mut self, mode: OpMode) -> bool`**
    Initializes hardware communication and puts the sensor into the requested fusion mode.
*   **`pub fn reset(&mut self) -> bool`**
    Issues a software reset to the IMU and restores the current configuration.

#### Data Fetching
*Methods return `Some(T)` on successful hardware reads, and `None` if an I2C bus error or lockup occurs.*

*   **`pub fn get_accelerometer(&mut self) -> Option<Vector3>`**
*   **`pub fn get_magnetometer(&mut self) -> Option<Vector3>`**
*   **`pub fn get_gyroscope(&mut self) -> Option<Vector3>`**
*   **`pub fn get_euler_angles(&mut self) -> Option<Vector3>`**
*   **`pub fn get_linear_acceleration(&mut self) -> Option<Vector3>`**
*   **`pub fn get_gravity(&mut self) -> Option<Vector3>`**
*   **`pub fn get_quaternion(&mut self) -> Option<Quaternion>`**
*   **`pub fn get_temperature(&mut self) -> Option<i8>`**
*   **`pub fn get_raw_sensor_data(&mut self) -> Option<RawSensorData>`**
    Reads all 9-axis raw values in a single 18-byte I2C burst transaction (utilizes `I2C_RDWR` if available).

#### Diagnostics & Utilities
*   **`pub fn get_calibration_status(&mut self) -> Option<CalibrationStatus>`**
*   **`pub fn get_diagnostics(&self) -> Diagnostics`**
*   **`pub fn to_euler_degrees(q: &Quaternion) -> Vector3`** (Static method on `BNO055`).

```rust
use libbno055::{BNO055, OpMode, Quaternion};

fn main() -> Result<(), &'static str> {
    let mut imu = BNO055::new_i2c(0x28, "/dev/i2c-1")?;
    if imu.begin(OpMode::NDOF) {
        if let Some(q) = imu.get_quaternion() {
            let euler = BNO055::to_euler_degrees(&q);
            println!("Roll: {:.2}, Pitch: {:.2}, Yaw: {:.2}", euler.x, euler.y, euler.z);
        }
    }
    Ok(())
}
```

---

## Class HeadingController

Heading PID & Feedforward Controller for robot straight-line driving & heading lock.

Header: `#include "libbno055-linux/controllers/heading_controller.hpp"`

```cpp
namespace bno055lib {

struct Quat {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

class HeadingController {
public:
    struct Config {
        double kp{0.05};             ///< Proportional Gain
        double ki{0.001};            ///< Integral Gain (Trapezoidal Rule)
        double kd{0.01};             ///< Derivative Gain (Filtered Gyro-based)
        double kff{0.0};             ///< Feedforward Gain
        double max_output{1.0};      ///< Max angular output limit
        double min_output{-1.0};     ///< Min angular output limit
        double max_i_term{0.2};      ///< Anti-windup saturation limit
        double deadband_deg{0.02};   ///< Micro-deadband (deg)
        double cutoff_freq_hz{20.0}; ///< Low-pass filter cutoff frequency (Hz)
    };

    struct Output {
        double correction{0.0};   ///< Total control output u = u_FF + u_PID
        double left_motor{0.0};   ///< Left wheel speed [0.0, 1.0]
        double right_motor{0.0};  ///< Right wheel speed [0.0, 1.0]
        double error_deg{0.0};     ///< Shortest heading error in degrees
        double gyro_filtered{0.0}; ///< Low-pass filtered gyro rate
    };

    HeadingController() noexcept;
    explicit HeadingController(const Config& config) noexcept;

    void setGains(double kp, double ki, double kd, double kff = 0.0) noexcept;
    void setConfig(const Config& config) noexcept;
    const Config& getConfig() const noexcept;
    void reset() noexcept;

    // Euler Degrees Update
    Output update(double target_heading_deg,
                  double current_heading_deg,
                  double dt,
                  double gyro_z_deg = 0.0,
                  double base_velocity = 0.5,
                  double target_yaw_rate_deg = 0.0) noexcept;

    // Direct Quaternion Update Overload
    Output update(const Quat& q_target,
                  const Quat& q_current,
                  double dt,
                  double gyro_z_deg = 0.0,
                  double base_velocity = 0.5,
                  double target_yaw_rate_deg = 0.0) noexcept;
};

// Utilities
double normalizeAngleDeg(double angle_deg) noexcept;
double fastExtractYawDeg(double qw, double qx, double qy, double qz) noexcept;
double fastExtractYawDeg(const Quat& q) noexcept;

}  // namespace bno055lib
```



