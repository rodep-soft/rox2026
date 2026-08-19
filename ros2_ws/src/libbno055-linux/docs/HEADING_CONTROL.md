# Robot Straight-Line & Heading PID Controller Architecture

This document provides a comprehensive guide to the **Heading Controller** in `libbno055-linux` (`bno055lib::HeadingController`), as well as its ROS 2 nodes (`bno055_heading_control_node` and `bno055_lifecycle_heading_control_node`).

---

## Control Architecture

The heading controller combines **Kinematic Feedforward (FF)** and **Feedback (PID)** control to maintain target heading and reduce straight-line drift.

```text
                                +-----------------------------------+
                                | Kinematic Feedforward (FF Term)   |
                                |       u_FF = K_ff * w_ref         |
                                +-----------------+-----------------+
                                                  |
                                                  v
+-------------------+   Angle Diff   +------------+------------+   u_total   +-------------------+
| Target Heading    | -------------> | PID Controller          | ----------> | Differential      |
| Quaternion / Deg  |  std::remainder| - P: Kp * error         |             | Motor Output      |
+-------------------+                | - I: Trapezoidal Rule   |             | (Left & Right)    |
                                     | - D: LPF Gyro (-Kd * w) |             +-------------------+
+-------------------+                +------------+------------+
| Current IMU Yaw   | ----------------------------+
| Quaternion / Deg  |  Fast Yaw Extract
+-------------------+
```

---

## Key Mathematical & Control Principles

### 1. Shortest-Path Angle Normalization ($\pm 180^\circ$ Wrapping)
Angle differences across the $\pm 180^\circ$ boundary are wrapped into $[-180^\circ, +180^\circ]$ degrees using `std::remainder`:

$$\text{Error} = \text{std::remainder}(\theta_{\text{target}} - \theta_{\text{current}}, 360.0)$$

This eliminates 360-degree boundary jump singularities and prevents reverse spin accidents.

### 2. Trapezoidal (Tustin) Rule Integration
Rather than standard Euler integration, the integral term uses the **Trapezoidal (Tustin) Rule** for superior mathematical accuracy under sampling time jitter ($\Delta t$):

$$I(t) = \text{clamp}\left( I(t-1) + K_i \cdot \frac{e(t) + e(t-1)}{2} \cdot \Delta t, \ -I_{\text{max}}, \ +I_{\text{max}} \right)$$

Anti-windup clamping (`max_i_term`) strictly caps the integral term to prevent saturation when wheels slip or stall.

### 3. 1st-Order Low-Pass Filtered Gyro Rate D-Term
To eliminate derivative noise amplification without lag, the controller feeds back the **1st-order Low-Pass Filtered (LPF) Gyro Rate** ($\omega_z$) directly:

$$\tau = \frac{1}{2\pi \cdot f_{\text{cutoff}}}, \quad \alpha = \frac{\Delta t}{\tau + \Delta t}$$
$$\omega_{\text{filtered}}(t) = \omega_{\text{filtered}}(t-1) + \alpha \cdot \left(\omega_z(t) - \omega_{\text{filtered}}(t-1)\right)$$
$$D(t) = -K_d \cdot \omega_{\text{filtered}}(t)$$

Mechanical motor vibration noise is filtered out while anti-slip counter-torque is applied instantaneously.

### 4. Micro-Deadband & Non-Linear Hunting Suppression
To eliminate high-frequency micro-hunting around zero error, errors below `deadband_deg` (default: $0.02^\circ$) are smoothed to $0.0^\circ$:

$$e(t) = \begin{cases} 0.0 & \text{if } |e(t)| < \text{deadband\_deg} \\ e(t) & \text{otherwise} \end{cases}$$

### 5. Slew-Rate Limiter (Angular Acceleration Constraint)
Prevents wheel slip and motor gear shock by constraining the maximum change rate of the output ($\text{rad/s}^2$):

$$u(t) = u(t-1) + \text{clamp}\left( u_{\text{raw}}(t) - u(t-1), \ -\text{slew\_rate} \cdot \Delta t, \ +\text{slew\_rate} \cdot \Delta t \right)$$

---

## C++ API Usage

Header: `#include "libbno055-linux/controllers/heading_controller.hpp"`

```cpp
#include "libbno055-linux/controllers/heading_controller.hpp"

bno055lib::HeadingController controller;

// Configuration
bno055lib::HeadingController::Config cfg;
cfg.kp = 0.05;
cfg.ki = 0.001;
cfg.kd = 0.01;
cfg.kff = 0.0;
cfg.cutoff_freq_hz = 20.0; // Low-pass filter cutoff frequency in Hz
cfg.max_slew_rate = 2.0;   // Max output change rate (rad/s^2)
cfg.max_i_term = 0.2;
controller.setConfig(cfg);

// Update using Quaternions
bno055lib::Quat q_target{1.0, 0.0, 0.0, 0.0};
bno055lib::Quat q_current{0.7071, 0.0, 0.0, 0.7071}; // 90 deg Yaw

auto out = controller.update(q_target, q_current, dt, gyro_z_deg, base_velocity);

// Access motor speeds [0.0, 1.0]
double left_speed = out.left_motor;
double right_speed = out.right_motor;
```

---

## ROS 2 Nodes & Fail-Safe Architecture

The package provides both **Standard Composable Components** and **Managed Lifecycle Nodes**.

### Safety Features
1. **Command-Loss Safety Watchdog**: If `cmd_vel_in` stops for more than `cmd_vel_timeout` (0.5s), the node automatically publishes Zero Velocity (`linear.x = 0, angular.z = 0`) to prevent runaway accidents.
2. **IMU Fail-Safe Passthrough**: If IMU data is lost or times out (`imu_timeout > 1.0s`), the node switches to **Passthrough Mode**, ensuring robot movement never stops.
3. **Zero-Copy Transport**: Registers as an `rclcpp_components` plugin for intra-process zero-copy memory transport.

---

## ROS 2 Parameter Reference (`config/heading_control_params.yaml`)

| Parameter | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `kp` | `double` | `0.05` | Proportional gain for heading error correction. |
| `ki` | `double` | `0.001` | Integral gain (Trapezoidal Rule integration). |
| `kd` | `double` | `0.01` | Derivative gain (Filtered Gyro-based). |
| `kff` | `double` | `0.0` | Kinematic Feedforward gain. |
| `max_i_term` | `double` | `0.2` | Anti-windup integral saturation limit. |
| `max_output` | `double` | `1.0` | Maximum correction velocity output (rad/s). |
| `deadband_deg` | `double` | `0.02` | Micro-deadband threshold (deg) to suppress hunting. |
| `cutoff_freq_hz` | `double` | `20.0` | Low-pass filter cutoff frequency for gyro rate (Hz). |
| `max_slew_rate` | `double` | `2.0` | Max change rate of output (rad/s^2) to prevent motor shock. |
| `cmd_vel_timeout` | `double` | `0.5` | Safety Watchdog timeout (s) for command loss. |
| `imu_timeout` | `double` | `1.0` | IMU connection loss timeout (s) for Fail-Safe Passthrough. |
| `angular_deadband` | `double` | `0.01` | Turning velocity threshold (rad/s) to unlock heading. |

---

## Quick Start Guide

```bash
# Composable Container Mode
ros2 launch libbno055_linux heading_control_launch.py

# Nav2 Lifecycle Manager Mode
ros2 launch libbno055_linux heading_control_launch.py node_type:=lifecycle
```
