# RobStride 05 bringup work status

Last updated: 2026-07-10

## Branch and commits

- Current branch: `jodan/topic_receive_logs`
- Published commit: `a034646 Log received control topics`
- This document and the additional message-content logging are pending commit and push.

## Bringup configuration

The launch file is `robot_bringup/launch/robstride05.launch.py`. It starts:

```text
joy_node (/joy)
  -> roller_position_controller (/robstride/position_cmd)
  -> robstride_position_node (/CAN/can0/transmit)
  -> ros2socketcan (/CAN/can0/transmit -> SocketCAN can0)
```

The YAML configuration is `robot_bringup/config/robstride05.yaml`.

- `roller_position_controller` now reads `enable_axis` (not the old `enable_button` name).
- The active configuration uses `enable_axis: 3`.
- `can_interface` must remain `can0` with the current YAML, because `can_tx_topic` is `/CAN/can0/transmit`.

## RDK hardware status

On the RDK, `can0` was checked and reported `UP, LOWER_UP`.

Start the bringup after building and sourcing the workspace:

```bash
cd ~/rox2026/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup robstride05.launch.py
```

Before starting, confirm the joystick device exists with `ls -l /dev/input/js0`. Use
`joy_dev:=/dev/input/jsN` when a different device number is assigned.

The launch enables the RobStride motor during startup. On `Ctrl-C`, it commands the configured
home position for `shutdown_return_wait_ms` (currently 7000 ms) and then sends the motor-stop
frame. Keep the mechanism clear before starting or stopping the launch.

## Logging changes

The nodes now log each received input message:

- `roller_angle_controller`: `/joy` topic name plus all `axes` and `buttons` values.
- `robstride_can_node`: `/robstride/position_cmd` topic name plus the received angle in radians.
- `ros2socketcan_bridge`: `/CAN/can0/transmit` topic name plus CAN ID, DLC, flags, and data bytes.

The former raw SocketCAN send/receive payload logs were removed before adding the receive-topic
log. The current SocketCAN log is emitted only when the ROS transmit topic is received.

## Verification

- `git diff --check`: passed.
- Docker build passed for `roller_angle_controller`, `robstride_can_node`,
  `ros2socketcan_bridge`, and `robot_bringup` after the topic-only logging change.
- Docker build passed for `roller_angle_controller`, `robstride_can_node`, and
  `ros2socketcan_bridge` after adding message-content logging.
- No CAN command was sent as part of the container verification.
