# Examples Overview & Directory Guide

This directory contains standalone application examples categorized by language, clearly decoupled from the core driver engine and official ROS 2 nodes.

## Directory Structure

```text
examples/
├── cpp/       # Native C++17 application examples
├── c/         # C99 API examples
└── python/    # Python 3 binding examples
```

## Summary of Examples

| File | Subdirectory | Language | Purpose & Features |
| :--- | :--- | :---: | :--- |
| **`heading_control_demo.cpp`** | `cpp/` | C++17 | 100Hz Straight-Line PID Heading Controller with ASCII dashboard demo. |
| **`read_all_data.cpp`** | `cpp/` | C++17 | Interactive dashboard reading all physical vectors (Accel, Gyro, Mag, Euler, Linear Accel, Gravity, Quaternion, Temp). |
| **`read_data_noexcept.cpp`** | `cpp/` | C++17 | Non-throwing `noexcept` API usage pattern suitable for hard real-time / safety-critical systems. |
| **`calibrate.cpp`** | `cpp/` | C++17 | Interactive sensor calibration helper & status logger. |
| **`benchmark_imu.cpp`** | `cpp/` | C++17 | Low-latency I2C/UART bus throughput and read timing benchmark tool. |
| **`c_demo.c`** | `c/` | C99 | C FFI API binding usage example for legacy C codebases. |
| **`python_demo.py`** | `python/` | Python 3 | High-level Python binding (`libbno055`) example with exception handling. |

---

## How to Build & Run Examples

### C++ / C Examples (CMake)

```bash
mkdir -p build && cd build
cmake -DENABLE_CLANG_TIDY=OFF ..
make -j$(nproc)

# Run the 100Hz Heading PID Control Demo
./heading_control_demo /dev/i2c-1

# Run the full sensor dashboard
./read_all_data /dev/i2c-1
```

### Python Example

```bash
pip install .
python3 examples/python/python_demo.py
```
