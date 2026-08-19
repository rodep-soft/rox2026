# libbno055-linux

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](include/libbno055-linux/bno055.hpp)
[![crates.io](https://img.shields.io/crates/v/libbno055.svg)](https://crates.io/crates/libbno055)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB.svg)](src/python/bindings.cpp)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted-22314E.svg)](docs/INTEGRATION.md)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04-E95420.svg)](docs/INTEGRATION.md)
[![CI](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml/badge.svg)](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml)
[![Coverage](https://codecov.io/gh/lazytatzv/libbno055-linux/graph/badge.svg)](https://codecov.io/gh/lazytatzv/libbno055-linux)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue.svg)](https://lazytatzv.github.io/libbno055-linux/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A high-performance, polyglot C++17, ROS 2, Python, C, and Rust driver for the Bosch BNO055 9-DOF IMU sensor on Linux systems.

---

## Highlights

- **Native I2C & UART Support**: Fast POSIX drivers with hardware auto-recovery for I2C bus lockups and UART overrun errors.
- **ROS 2 Lifecycle & Diagnostics**: Production ROS 2 lifecycle nodes with zero-copy publishing and diagnostic streams.
- **Multi-Language FFI Bindings**: C++17 core engine with native C, Python (`pip`), and safe Rust (`crates.io`) interfaces.

---

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

---

## Features

- **Polyglot Bindings**: First-class support for C++17, ROS 2, Python (`pip`), Rust (`crates.io`), and native C.
- **Zero-Allocation Hot Path**: Memory-optimized, no-heap allocations during high-rate sensor readouts.
- **Robust Hardware Recovery**: Auto-reconnects on I2C `EIO` bus lockups, clock-stretching glitches, and UART overrun errors.
- **High-Throughput Burst Reads**: 18-Byte sequential I2C/UART burst reads (~450µs at 400kHz I2C).
- **Linux GPIO Interrupt (IRQ)**: Sub-millisecond latency via hardware INT pin edge detection using POSIX `poll()`.

---

## Quick Start by Language

### Rust (`crates.io`)

```bash
cargo add libbno055
```

```rust
use libbno055::{BNO055, OpMode};

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

### Python (`pip`)

```bash
pip install libbno055-linux  # Or: pip install . (from source)
```

```python
import libbno055

imu = libbno055.BNO055(address=0x28, device="/dev/i2c-1")
if imu.begin(libbno055.OpMode.NDOF):
    q = imu.get_quaternion()
    if q:
        euler = libbno055.to_euler_degrees(q)
        print(f"Roll: {euler.x:.2f}, Pitch: {euler.y:.2f}, Yaw: {euler.z:.2f}")
```

---

### C++17 (Native CMake)

```cpp
#include <libbno055-linux/bno055.hpp>
#include <iostream>

int main() {
    bno055lib::BNO055 imu(0x28, "/dev/i2c-1");
    if (imu.begin(bno055lib::OpMode::NDOF)) {
        if (auto q = imu.getQuaternionNoexcept()) {
            auto euler = bno055lib::toEulerDegrees(*q);
            std::cout << "Roll: " << euler.x << " Pitch: " << euler.y << " Yaw: " << euler.z << "\n";
        }
    }
}
```

---

### ROS 2 Node (Binary Installation)

```bash
sudo apt update && sudo apt install ros-${ROS_DISTRO}-libbno055-linux
ros2 launch libbno055_linux bno055_launch.py
```

---

## Documentation

- **[API Reference](https://github.com/lazytatzv/libbno055-linux/blob/main/docs/API_REFERENCE.md)**: Full class and function reference for C++, C, Python, and Rust.
- **[Integration & Tuning Guide](https://github.com/lazytatzv/libbno055-linux/blob/main/docs/INTEGRATION.md)**: ROS 2 YAML parameters, EKF setup, 400kHz I2C, UART 921600 bps tuning, and Rust integration.
- **[Architecture & Design](https://github.com/lazytatzv/libbno055-linux/blob/main/docs/ARCHITECTURE.md)**: PIMPL design, zero-copy transport, FFI layers, and state machines.
- **[Troubleshooting & FAQ](https://github.com/lazytatzv/libbno055-linux/blob/main/docs/TROUBLESHOOTING.md)**: Hardware wiring, permissions, and clock-stretching fixes.

---

## License

This project is released under the [MIT License](https://github.com/lazytatzv/libbno055-linux/blob/main/LICENSE).
