# ROS 2 Node / Topic Mermaid

対象: `ros2_ws/src`

ブランチ: `jodan/test_can_stm`

## 現在の主な流れ

```mermaid
flowchart LR
  joy["joy_node<br/>pub /joy<br/>sensor_msgs/msg/Joy"]

  roller_ctrl["roller_controller_node<br/>sub /joy<br/>pub /roller/rpm_value"]
  mad_motor["belt_controller_node<br/>sub /joy<br/>pub /belt/rpm_value"]

  roller_cmd["roller_can_command_node<br/>sub /roller/rpm_value<br/>pub /roller/can_frame"]
  belt_cmd["belt_can_command_node<br/>sub /belt/rpm_value<br/>pub /belt/can_frame"]

  packer["motor_can_packer_node<br/>sub /roller/can_frame<br/>sub /belt/can_frame<br/>pub /CAN/can0/transmit"]
  socketcan["ros2socketcan<br/>sub /CAN/can0/transmit<br/>pub /CAN/can0/receive"]
  can0["SocketCAN can0"]

  joy -->|/joy| roller_ctrl
  joy -->|/joy| mad_motor

  roller_ctrl -->|/roller/rpm_value<br/>std_msgs/msg/Int16| roller_cmd
  mad_motor -->|/belt/rpm_value<br/>std_msgs/msg/Int16| belt_cmd

  roller_cmd -->|/roller/can_frame<br/>can_msgs/msg/Frame| packer
  belt_cmd -->|/belt/can_frame<br/>can_msgs/msg/Frame| packer

  packer -->|/CAN/can0/transmit<br/>can_msgs/msg/Frame| socketcan
  socketcan --> can0
```

## roller_position.launch.py の起動構成

```mermaid
flowchart LR
  launch["roller_position.launch.py"]
  config["roller_position.yaml"]

  packer["motor_can_packer_node"]
  socketcan["ros2socketcan"]
  joy["joy_node"]
  mad_motor["belt_controller_node"]
  roller_cmd["roller_can_command_node<br/>executable motor_can_command_node"]
  belt_cmd["belt_can_command_node<br/>executable motor_can_command_node"]

  launch --> config
  config --> packer
  config --> socketcan
  config --> joy
  config --> mad_motor
  config --> roller_cmd
  config --> belt_cmd

  launch --> packer
  launch --> socketcan
  launch --> joy
  launch --> mad_motor
  launch --> roller_cmd
  launch --> belt_cmd
```

## STM Roller / Belt Packer

```mermaid
flowchart LR
  roller_rpm["/roller/rpm_value<br/>std_msgs/msg/Int16"]
  belt_rpm["/belt/rpm_value<br/>std_msgs/msg/Int16"]

  roller_cmd["roller_can_command_node"]
  belt_cmd["belt_can_command_node"]

  roller_frame["/roller/can_frame<br/>can_msgs/msg/Frame"]
  belt_frame["/belt/can_frame<br/>can_msgs/msg/Frame"]

  packer["motor_can_packer_node"]
  out["/CAN/can0/transmit<br/>can_msgs/msg/Frame<br/>can_id 0x201<br/>dlc 4"]
  socketcan["ros2socketcan"]

  roller_rpm --> roller_cmd
  belt_rpm --> belt_cmd
  roller_cmd --> roller_frame
  belt_cmd --> belt_frame
  roller_frame -->|output data 2..3| packer
  belt_frame -->|output data 0..1| packer
  packer --> out
  out --> socketcan
```

## 全体の周辺系統

```mermaid
flowchart LR
  joy["joy_node<br/>/joy"]

  base["base_controller"]
  mecanum["mecanum_controller_node"]
  wheel["wheel_to_can_node"]

  roller_position["roller_position_controller"]
  rob_pos["robstride_position_node"]
  rob_vel_src["external node<br/>/robstride/velocity_cmd"]
  rob_vel["robstride_velocity_node"]

  roller_ctrl["roller_controller_node"]
  mad_motor["belt_controller_node"]
  roller_cmd["roller_can_command_node"]
  belt_cmd["belt_can_command_node"]
  packer["motor_can_packer_node"]

  socketcan["ros2socketcan"]
  can0["SocketCAN can0"]

  joy --> base
  base -->|/cmd_vel| mecanum
  mecanum -->|/motor_vel| wheel
  wheel -->|/CAN/can0/transmit| socketcan

  joy --> roller_position
  roller_position -->|/robstride/position_cmd| rob_pos
  rob_pos -->|/CAN/can0/transmit| socketcan
  rob_vel_src --> rob_vel
  rob_vel -->|/CAN/can0/transmit| socketcan

  joy --> roller_ctrl
  joy --> mad_motor
  roller_ctrl -->|/roller/rpm_value| roller_cmd
  mad_motor -->|/belt/rpm_value| belt_cmd
  roller_cmd -->|/roller/can_frame| packer
  belt_cmd -->|/belt/can_frame| packer
  packer -->|/CAN/can0/transmit| socketcan

  socketcan --> can0
```

## 注意点

- `belt_controller_node` は `/belt/rpm_value` を publish するように変更済み。
- `belt_can_command_node` は `/belt/rpm_value` を subscribe するため、MAD motor のPWM値が belt 側CAN frameへつながる。
- `roller_controller_node` は `/roller/rpm_value` を publish する実装だが、`angle_motor` のCMakeにはまだ実行ファイル登録がない。
- `roller_position.launch.py` では `belt_controller_node` は起動するが、`roller_controller_node` は起動しない。
- `motor_can_packer_node` は有効な入力channelをすべて受け取るまで `/CAN/can0/transmit` にpublishしない。
