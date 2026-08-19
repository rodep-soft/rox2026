# Features and Comparison

## Why libbno055-linux?

| Feature | `libbno055-linux` | Adafruit BNO055 | Bosch Reference |
| :--- | :---: | :---: | :---: |
| **Linux I2C & UART** | ✅ | ⚠️ | ❌ |
| **Automatic Recovery** | ✅ | ❌ | ❌ |
| **ROS 2 & Lifecycle** | ✅ | ❌ | ❌ |
| **Zero-Allocation Hot Path** | ✅ | ❌ | ❌ |
| **Rust & Python Bindings** | ✅ | ⚠️ | ❌ |
| **Burst Read (18-Byte)** | ✅ | ❌ | ❌ |
| **GPIO IRQ Latency** | ✅ | ❌ | ❌ |

### Userspace vs. Kernel IIO Driver

While the mainline Linux kernel includes an IIO driver (`drivers/iio/imu/bno055`) starting from Linux 6.1, `libbno055-linux` is implemented as a **Userspace Driver** to address specific integration requirements in robotics (e.g., ROS 2) and rapid prototyping:

1. **Zero-Configuration Setup**: No need to compile kernel modules or write Device Tree Overlays (DTO). Installable via `pip install`, `cargo add`, or `apt install`, and operates on any standard I2C/UART port.
2. **`I2C_RDWR` Utilization**: Uses `ioctl(I2C_RDWR)` for Repeated Start burst-reading. It achieves low latency (~450µs for 18-bytes) in userspace by reducing system calls.
3. **Interrupt Driven**: Uses Linux `poll()` on sysfs GPIOs to wait for hardware INT pin edges in a background thread, achieving CPU-free idling similar to kernel-triggered buffers.
4. **Native ROS 2 Integration**: Directly publishes Zero-Copy `sensor_msgs/Imu`, avoiding the overhead of parsing multiple kernel sysfs text files (`in_accel_x_raw`, etc.) in a polling loop.

## Core Features
- **Polyglot Bindings**: First-class support for C++17, ROS 2, Python (`pip`), Rust (`crates.io`), and native C.
- **Zero-Allocation Hot Path**: Memory-optimized, no-heap allocations during high-rate sensor readouts.
- **Robust Hardware Recovery**: Auto-reconnects on I2C `EIO` bus lockups, clock-stretching glitches, and UART overrun errors.
- **High-Throughput Burst Reads**: 18-Byte sequential I2C/UART burst reads (~450µs at 400kHz I2C).
- **Linux GPIO Interrupt (IRQ)**: Sub-millisecond latency via hardware INT pin edge detection using POSIX `poll()`.
