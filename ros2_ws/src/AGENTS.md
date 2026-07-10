# ROS 2 Workspace Contributor Guide

## Scope

This directory is the `src/` tree of a ROS 2 workspace. Each first-level package is an ament/CMake package.

- Node implementations: `<package>/src/` and `<package>/include/`
- ROS parameters: `<package>/config/`
- Multi-node launch configurations: `robot_bringup/launch/` and `robot_bringup/config/`
- Package metadata and dependencies: `<package>/package.xml` and `<package>/CMakeLists.txt`

Read the package README before changing a package-specific protocol or motor behavior. The workspace-level architecture is documented in `README.md` and `../README.md`.

## Package Roles

| Package | Main executable/node | Role |
|---|---|---|
| `robot_bringup` | launch files only | Starts a compatible set of controller, command, and CAN transport nodes. |
| `roller_angle_controller` | `roller_angle_controller_node` / `roller_position_controller` | Converts `/joy` inputs into preset RobStride position commands on `/robstride/position_cmd`. |
| `robstride_can_node` | `robstride_can_node` / `robstride_position_node` | Converts RobStride position commands into extended `can_msgs/msg/Frame` messages for the SocketCAN transmit topic. |
| `ros2socketcan_bridge` | `ros2socketcan` | Bridges `CAN/<interface>/transmit` and `CAN/<interface>/receive` ROS topics to a Linux SocketCAN interface. |
| `belt_controller` | `belt_controller_node` | Converts `/joy` inputs into the belt PWM command topic `/belt/rpm_value`. |
| `motor_can_bridge` | `motor_can_command_node` | Converts one `std_msgs/msg/Int16` PWM input into a CAN frame; sends zero after its configured input timeout. |
| `motor_can_bridge` | `motor_can_packer_node` | Combines the first two bytes from roller and belt CAN-frame topics into one STM CAN frame. |
| `mecanum_controller` | `base_controller_node` | Converts `/joy` into `/cmd_vel` (`geometry_msgs/msg/Twist`). |
| `mecanum_controller` | `mecanum_controller_node` | Converts `/cmd_vel` into four wheel speeds on `/motor_vel`. |
| `wheel_to_can` | `wheel_to_can_node` | Converts the four `/motor_vel` values into RobStride velocity-mode CAN frames for four wheel motors. It also enables and stops those motors. |
| `el05_driver` | `usb_can_analyzer_node` | Drives a Seeed USB-CAN Analyzer over serial, using the package-specific `/ssuca/transmit` and `/ssuca/receive` message interface. |

## Node Flows

### RobStride 05 roller-angle bringup

`robot_bringup/launch/robstride05.launch.py` starts this chain:

```text
joy_node
  /joy (sensor_msgs/msg/Joy)
  -> roller_position_controller
  /robstride/position_cmd (std_msgs/msg/Float32, radians)
  -> robstride_position_node
  /CAN/can0/transmit (can_msgs/msg/Frame, extended ID)
  -> ros2socketcan
  -> SocketCAN can0 -> RobStride 05
```

`roller_position_controller` publishes its configured storage, intake, or shoot angle only when the configured enable axis/button condition is met. `robstride_position_node` clamps the target position, sets up PP position mode at startup, and repeatedly publishes `loc_ref` frames. Its shutdown path sends the RobStride motor-stop frame.

### PWM-to-STM CAN path

The generic command and packing nodes form this path when configured together:

```text
controller node
  /roller/rpm_value or /belt/rpm_value (std_msgs/msg/Int16)
  -> roller_can_command_node or belt_can_command_node
  /roller/can_frame or /belt/can_frame (can_msgs/msg/Frame)
  -> motor_can_packer_node
  /CAN/can0/transmit (can_msgs/msg/Frame)
  -> ros2socketcan -> SocketCAN -> STM
```

`motor_can_command_node` owns one PWM input and encodes it into bytes 0-1 of a CAN frame. `motor_can_packer_node` places belt data in bytes 0-1 and roller data in bytes 2-3 of the final 4-byte STM frame. Do not connect both this path and an unrelated publisher to the same transmit topic without confirming CAN IDs and payload ownership.

### Mecanum wheel path

`robot_bringup/launch/base.launch.py` starts the base-control chain:

```text
joy_node
  /joy (sensor_msgs/msg/Joy)
  -> base_controller_node
  /cmd_vel (geometry_msgs/msg/Twist)
  -> mecanum_controller_node
  /motor_vel (std_msgs/msg/Float32MultiArray, four wheel velocities)
  -> wheel_to_can_node
  CAN/can0/transmit (can_msgs/msg/Frame, extended ID)
  -> ros2socketcan -> SocketCAN -> four wheel motors
```

`wheel_to_can_node` is a separate RobStride velocity-mode command path with hard-coded motor IDs and startup/shutdown frames. Treat it as hardware-specific; do not assume its protocol parameters are shared with `robstride_can_node`.

### USB-CAN Analyzer path

`usb_can_analyzer_node` is an alternative hardware transport, not a `can_msgs/msg/Frame` SocketCAN bridge:

```text
/ssuca/transmit (seeed_usb_can_analyzer_driver/msg/CanFrame)
  -> usb_can_analyzer_node -> USB serial CAN analyzer -> CAN bus
CAN bus -> USB serial CAN analyzer -> usb_can_analyzer_node
  -> /ssuca/receive (seeed_usb_can_analyzer_driver/msg/CanFrame)
```

## Build And Test

Always build and test inside the Docker environment defined by `../../Dockerfile` and `../../docker-compose.yml`. Do not run `colcon` on the host.

Start the container from the repository root (`../../` from this directory):

```bash
docker compose up -d --build
```

Then build and test from `/root/ros2_ws` inside the `ros2_rox2026` container:

```bash
docker compose exec ros2_rox2026 bash -lc \
  'source /opt/ros/humble/setup.bash && cd /root/ros2_ws && \
   colcon build --packages-select <package_name> && \
   source install/setup.bash && \
   colcon test --packages-select <package_name> && \
   colcon test-result --verbose'
```

Build only the affected package unless an interface, shared message, or launch/configuration contract changes. Do not modify or remove existing workspace build artifacts to resolve an error.

## C++ Conventions

- Use C++17 and retain the existing ament/CMake structure.
- Keep compiler warnings enabled: `-Wall -Wextra -Wpedantic`.
- Do not prefix constants with `k`; name them directly in the local style, for example `CommTypeEnable` rather than `kCommTypeEnable`.
- Add dependencies consistently in both `package.xml` and `CMakeLists.txt`.
- Use existing ROS 2 patterns: parameters are declared before reading, publishers/subscriptions use `rclcpp`, and timers own periodic output.
- Keep CAN frame packing explicit about byte order, IDs, DLC, and extended-frame flags.

## ROS Configuration

- Treat topic names, node names, parameter keys, and YAML files as public runtime interfaces. Update all launch/configuration consumers when changing one.
- Launch-time configurations belong in `robot_bringup` when they combine multiple packages. Do not add package-local launch files solely to duplicate a bringup launch.
- Keep YAML node keys aligned with the runtime node name passed to `rclcpp::Node`.
- Document new or changed parameters in the relevant package README and configuration comments.

## Motor And CAN Safety

- Do not alter CAN IDs, payload layouts, control modes, enable/disable frames, or timeout behavior without checking the hardware protocol documentation and every affected launch configuration.
- Preserve explicit safe-stop behavior. For a new motor command path, define what happens on startup, stale commands, and process shutdown.
- Avoid sending real CAN commands during ordinary tests. Prefer compile checks and ROS-topic-level verification unless hardware testing is explicitly requested.

## Change Discipline

- Keep changes scoped to the requested package and runtime behavior.
- Do not discard unrelated uncommitted changes.
- Run `git diff --check` after edits. Include the verification result and any unavailable ROS dependencies or hardware-test limitations in the final report.

## Edulite 05 reference
https://wiki.aifitlab.com/robstride-docs/edulite-05-instruction-manualこれをよんでこの設計に合わせるときは合わせること。
