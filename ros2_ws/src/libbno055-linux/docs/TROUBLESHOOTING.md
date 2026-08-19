# Troubleshooting & FAQ

## I2C Permission Denied
**Error**: `Failed to open I2C device: /dev/i2c-1 (Permission denied)`
**Cause**: The user running the binary does not have access to the hardware I2C bus.
**Solution**:
Add your user to the `i2c` group:
```bash
sudo usermod -aG i2c $USER
```
Then log out and log back in.

## BNO055 Clock Stretching (I2C Lockups on Raspberry Pi)
**Error**: `[Warning] Temporary communication dropout` appears frequently.
**Cause**: The Broadcom I2C hardware on Raspberry Pi has a known bug handling the BNO055's clock stretching, causing the kernel driver to timeout.
**Solution**:
1. Lower the I2C baudrate. Edit `/boot/firmware/config.txt` (or `/boot/config.txt`) and set:
   ```text
   dtparam=i2c_arm=on,i2c_arm_baudrate=10000
   ```
   (The BNO055 supports up to 400kHz, but 10kHz to 50kHz is significantly more stable on the Raspberry Pi).
2. Reboot the Pi.

## Hardware Lockups or Unresponsive Sensor
**Error**: `[Error] I2C read failed continuously` or no data is being published, even after Auto-Recovery kicks in.
**Cause**: The BNO055 internal microcontroller occasionally crashes or permanently locks the I2C/UART bus when exposed to severe voltage spikes or EM interference.
**Solution**:
1. You can force a complete software hardware-reset without rebooting your computer by calling the `~/reset` service:
   ```bash
   ros2 service call /bno055_publisher_node/reset std_srvs/srv/Trigger
   ```
2. This invokes the `SYS_TRIGGER` register, power cycles the internal logic, and restores your previous operating mode.

## Sensor Calibration is Lost on Reboot
**Error**: The heading (Yaw) drifts immediately after power-on.
**Cause**: The BNO055 does not have internal non-volatile memory for calibration profiles. It loses its calibration every time it loses power.
**Solution**:
Use the provided `calibrate_imu` example to generate a `bno055_calib.bin` file, then load it in your code immediately after `begin()`:
```cpp
imu.loadCalibrationFile("bno055_calib.bin");
```

## Why isn't the Magnetometer working indoors?
**Error**: Calibration status for `Mag` is always 0 or 1, and heading is highly erratic.
**Cause**: Indoor environments with heavy metal structures, motors, and power lines cause severe magnetic distortion.
**Solution**:
Do not use `NDOF` mode indoors. Switch to `IMUPlus` mode:
```cpp
imu.begin(bno055lib::OpMode::IMUPlus);
```
`IMUPlus` mode ignores the magnetometer and provides a relative heading using only the high-precision gyroscope and accelerometer.

---

## ROS 2 Specific Troubleshooting

### ROS 2 Launch fails to find package share directory
**Error**: `package 'libbno055_linux' not found` or similar.
**Cause**: The workspace has not been sourced, or the package has not been built using `colcon`.
**Solution**:
1. Verify that your terminal has sourced the workspace setup script:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
2. Re-build the workspace and ensure `colcon` successfully builds the package:
   ```bash
   colcon build --packages-select libbno055_linux
   ```

### Intra-Process Zero-Copy Communication is not taking effect
**Error**: High CPU/Memory overhead still present; pointers are not being passed directly.
**Cause**: For ROS 2 to optimize message passing via zero-copy, the publishing node and the subscribing node must:
1. Be loaded into the **same component container** (single process).
2. Utilize `std::unique_ptr` and `std::move()` for publishing (which `bno055_publisher_node` does by default).
3. Both have `use_intra_process_comms` option enabled.
**Solution**:
When creating your pipeline, compose the BNO055 component and your subscriber component inside a single container executable (using `rclcpp_components`) and set `use_intra_process_comms` to `True` in the launch parameters.

### Lifecycle Node fails to publish data immediately
**Error**: The lifecycle node starts up, but `ros2 topic echo /imu/data` outputs nothing.
**Cause**: ROS 2 Lifecycle Nodes start in the `Unconfigured` state. They do not start the timer or open hardware connections until transitioned.
**Solution**:
Trigger the state transitions using the ROS 2 lifecycle CLI:
```bash
ros2 lifecycle set /bno055_lifecycle_publisher_node configure
ros2 lifecycle set /bno055_lifecycle_publisher_node activate
```
You can verify the current node state with:
```bash
ros2 lifecycle get /bno055_lifecycle_publisher_node
```

---

## C++ Standard & Legacy Environment Compatibility

### Compilation Error with C++11 / C++14
**Error**: `error: ‘optional’ in namespace ‘std’ does not name a type` or `error: ‘string_view’ in namespace ‘std’ does not name a type`
**Cause**: The native C++ API (`bno055.hpp`) uses modern C++17 features (`std::optional`, `std::string_view`, structured bindings).
**Solution**:
1. **Target C++17 in your C++ project**:
   Set `set(CMAKE_CXX_STANDARD 17)` in your `CMakeLists.txt` or pass `-std=c++17` to GCC/Clang.
2. **Use the C API (`bno055_c.h`) for C99 / C++11 / C++14 projects**:
   If your main application cannot be compiled with C++17, build `libbno055-linux` as a shared library (`libbno055-linux.so`) using a C++17 compiler once, then include `libbno055-linux/bno055_c.h` in your project. The C API provides a pure C99 FFI interface (`bno055_handle_t`) compatible with legacy compilers, C, Python, Rust, and Go.

