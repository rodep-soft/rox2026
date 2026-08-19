# ROS 2 Official Nodes & Components Directory

This directory contains the production ROS 2 driver nodes and application components for `libbno055-linux`.

## ROS 2 Node Architecture Index

| Component Category | Standard Composable Node | Managed Lifecycle Node | Description |
| :--- | :--- | :--- | :--- |
| **IMU Driver Publisher** | **`bno055_publisher_node`**<br>(`ros2_publisher_node.cpp`) | **`bno055_lifecycle_publisher_node`**<br>(`ros2_lifecycle_publisher_node.cpp`) | Reads BNO055 hardware over I2C/UART and publishes `/imu/data` & `/diagnostics`. |
| **Heading PID Corrector** | **`bno055_heading_control_node`**<br>(`ros2_heading_control_node.cpp`) | **`bno055_lifecycle_heading_control_node`**<br>(`ros2_lifecycle_heading_control_node.cpp`) | Subscribes to `cmd_vel_in` & `/imu/data` to output IMU-corrected `cmd_vel` with Fail-Safe Passthrough. |

---

## Which Node Architecture Should I Use?

- **Standard Composable Nodes (Recommended for General Use)**:
  - Immediate, friction-free startup without requiring a ROS 2 Lifecycle Manager.
  - Zero-Copy intra-process communication inside `component_container_mt`.
- **Managed Lifecycle Nodes (For Nav2 Lifecycle Systems)**:
  - Integrates directly with Nav2 `lifecycle_manager` state transitions (`unconfigured` -> `inactive` -> `active` -> `finalized`).

---

## Quick Start (One-Command Launch)

```bash
# 1. Standard Composable Container Mode (Zero-Copy Enabled)
ros2 launch libbno055_linux heading_control_launch.py

# 2. Managed Lifecycle Mode (Nav2 System Compatible)
ros2 launch libbno055_linux heading_control_launch.py node_type:=lifecycle
```
