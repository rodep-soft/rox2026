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
  roller_ctrl["roller_controller_node"]
  roller_cmd["roller_can_command_node<br/>executable motor_can_command_node"]
  belt_cmd["belt_can_command_node<br/>executable motor_can_command_node"]

  launch --> config
  config --> packer
  config --> socketcan
  config --> joy
  config --> mad_motor
  config --> roller_ctrl
  config --> roller_cmd
  config --> belt_cmd

  launch --> packer
  launch --> socketcan
  launch --> joy
  launch --> mad_motor
  launch --> roller_ctrl
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

## Launch file 一覧

```mermaid
flowchart LR
  roller_launch["roller_angle_controller<br/>roller_position.launch.py"]
  motor_command_launch["motor_can_bridge<br/>motor_can_command.launch.py"]
  motor_packer_launch["motor_can_bridge<br/>motor_can_packer.launch.py"]
  mabuchi_launch["motor_can_bridge<br/>mabuchi_can_command.launch.py"]
  belt_launch["motor_can_bridge<br/>mad_motor_can_command.launch.py"]
  robstride_all_launch["robstride_can_node<br/>robstride_can_node.launch.py"]
  robstride_pos_launch["robstride_can_node<br/>robstride_position.launch.py"]
  robstride_vel_launch["robstride_can_node<br/>robstride_velocity.launch.py"]
  base_launch["robot_bringup<br/>base.launch.py"]
  mecanum_launch["mecanum_controller<br/>mecanum.launch.py"]
  usbcan_launch["el05_driver<br/>usb_can_analyzer.launch.py"]
```

## roller_position.launch.py

現在の STM roller/belt 送信系をまとめて起動する launch。

```mermaid
flowchart LR
  launch["roller_position.launch.py"]
  config["roller_position.yaml"]

  joy["joy_node<br/>pub /joy"]
  belt_controller["belt_controller_node<br/>sub /joy<br/>pub /belt/rpm_value"]
  roller_cmd["roller_can_command_node<br/>sub /roller/rpm_value<br/>pub /roller/can_frame"]
  belt_cmd["belt_can_command_node<br/>sub /belt/rpm_value<br/>pub /belt/can_frame"]
  packer["motor_can_packer_node<br/>sub /roller/can_frame<br/>sub /belt/can_frame<br/>pub /CAN/can0/transmit"]
  socketcan["ros2socketcan<br/>sub /CAN/can0/transmit"]
  roller_controller["roller_controller_node<br/>sub /joy<br/>pub /roller/rpm_value"]

  launch --> config
  launch --> joy
  launch --> belt_controller
  launch --> roller_controller
  launch --> roller_cmd
  launch --> belt_cmd
  launch --> packer
  launch --> socketcan

  config --> belt_controller
  config --> roller_controller
  config --> roller_cmd
  config --> belt_cmd
  config --> packer
  config --> socketcan
  config --> joy

  joy -->|/joy| belt_controller
  joy -->|/joy| roller_controller
  belt_controller -->|/belt/rpm_value| belt_cmd
  roller_controller -->|/roller/rpm_value| roller_cmd
  roller_cmd -->|/roller/can_frame| packer
  belt_cmd -->|/belt/can_frame| packer
  packer -->|/CAN/can0/transmit| socketcan
```

## motor_can_bridge launch

```mermaid
flowchart LR
  command_launch["motor_can_command.launch.py"]
  command_config["motor_can_bridge/config/config.yaml"]
  roller_cmd["roller_can_command_node<br/>executable motor_can_command_node"]
  belt_cmd["belt_can_command_node<br/>executable motor_can_command_node"]

  packer_launch["motor_can_packer.launch.py"]
  packer["motor_can_packer_node"]

  mabuchi_launch["mabuchi_can_command.launch.py"]
  mabuchi_only["roller_can_command_node"]

  belt_launch["mad_motor_can_command.launch.py"]
  belt_only["belt_can_command_node"]

  command_launch --> command_config
  command_launch --> roller_cmd
  command_launch --> belt_cmd

  packer_launch --> command_config
  packer_launch --> packer

  mabuchi_launch --> command_config
  mabuchi_launch --> mabuchi_only

  belt_launch --> command_config
  belt_launch --> belt_only
```

## robstride_can_node launch

```mermaid
flowchart LR
  config["robstride_can_node/config/config.yaml"]

  all_launch["robstride_can_node.launch.py"]
  pos_launch["robstride_position.launch.py"]
  vel_launch["robstride_velocity.launch.py"]

  pos["robstride_position_node<br/>sub /robstride/position_cmd<br/>pub /CAN/can0/transmit"]
  vel["robstride_velocity_node<br/>sub /robstride/velocity_cmd<br/>pub /CAN/can0/transmit"]
  socketcan["ros2socketcan<br/>external launch needed"]

  all_launch --> config
  all_launch --> pos
  all_launch --> vel

  pos_launch --> config
  pos_launch --> pos

  vel_launch --> config
  vel_launch --> vel

  pos -->|/CAN/can0/transmit| socketcan
  vel -->|/CAN/can0/transmit| socketcan
```

## robot_bringup/base.launch.py

足回り系をまとめて起動する launch。`wheel_to_can_node` は `TimerAction` で5秒遅延起動される。

```mermaid
flowchart LR
  launch["robot_bringup/base.launch.py"]
  joy["joy_node<br/>pub /joy"]
  base["base_controller_node<br/>sub /joy<br/>pub /cmd_vel"]
  mecanum["mecanum_controller_node<br/>sub /cmd_vel<br/>pub /motor_vel"]
  wheel["wheel_to_can_node<br/>sub /motor_vel<br/>pub /CAN/can0/transmit"]
  socketcan["ros2socketcan<br/>sub /CAN/can0/transmit"]

  launch --> joy
  launch --> base
  launch --> mecanum
  launch --> wheel
  launch --> socketcan

  joy -->|/joy| base
  base -->|/cmd_vel| mecanum
  mecanum -->|/motor_vel| wheel
  wheel -->|/CAN/can0/transmit| socketcan
```

## その他の launch

```mermaid
flowchart LR
  mecanum_launch["mecanum.launch.py"]
  mecanum["mecanum_controller_node"]
  base_bad["base_controller<br/>注意: CMake上は base_controller_node"]

  usb_launch["usb_can_analyzer.launch.py"]
  usb_node["usb_can_analyzer_node<br/>sub /ssuca/transmit<br/>pub /ssuca/receive"]

  mecanum_launch --> mecanum
  mecanum_launch --> base_bad

  usb_launch --> usb_node
```

## 注意点

- `belt_controller_node` は `/belt/rpm_value` を publish するように変更済み。
- `belt_can_command_node` は `/belt/rpm_value` を subscribe するため、MAD motor のPWM値が belt 側CAN frameへつながる。
- `roller_controller_node` は `angle_motor` の実行ファイルとして登録済み。
- `roller_position.launch.py` では `belt_controller_node` と `roller_controller_node` の両方を起動する。
- `motor_can_packer_node` は有効な入力channelをすべて受け取るまで `/CAN/can0/transmit` にpublishしない。
- `mecanum_controller/launch/mecanum.launch.py` は `base_controller` を指定しているが、CMakeでinstallされる実行ファイルは `base_controller_node`。
