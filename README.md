# rox2026
ROX2026用レポジトリ

## Branch Rules

- `main`への直接pushは禁止(PR only) <== Ruleset入れてあるので大会前に削除
- Merge前にビルドチェックを行う
- branchを必ず切って作業する

---
## RDKX5
### ROS2パッケージ一覧

| Package | 説明 |
|---------|------|
| mecanum_controller | 足回りのモーター速度の計算  |
| wheel_to_can | 足回りeduliteのcanframe生成 |
| motor_can_bridge | stm32向けにMAD用のcan frame生成とroller/belt frameの集約 |
| ros2socketcan_bridge | canframeからcanデータ生成と送受信 |

---

### Topic一覧

| topic名 | 型 | publishing_node | subscribed_node |
| --------|----|-----|-----|
| /joy | sensor_msgs/msg/joy | joy_node | - |
| /cmd_vel | geometory_msgs/msg/Twist | base_controller | mecanum_controller_node
| /motor_vel | std_msgs/msg/Float32MultiArray | mecanum_controller_node | wheel_to_can_node |
| /CAN/can0/transmit | can_msgs/msg/Frame | wheel_to_can_node, motor_can_packer_node | *ros2socketcan* |
| /mad_motor/pwm_value | - | - | - |
| /can_tx | can_msgs/msg/Frame | motor_can_command_node | - |
| /roller/can_frame | can_msgs/msg/Frame | roller_controller_node | motor_can_packer_node |
| /belt/can_frame | can_msgs/msg/Frame | belt_controller_node | motor_can_packer_node |

*斜体は外部パッケージのノード

詳細： `docs/topic.md`

---

### コントローラボタン配置

| ボタン名 | 機能 |
| --------|------|
| left_stick | ロボットの移動のx,y軸 |
| right_stick | ロボットの旋回 |
| L2 | ばね発射，ベルト投射の安全装置キー |
| R1 | ばね投射の発射キー |
| △ | ベルト投射の発射キー |
| 〇 | - |
| ✕ | - |

## stm32f303k8

### 接続するセンサー,アクチュエータ一覧

| センサ，アクチュエータ名 | 役割 |
| -----------------------|------|
| encoder0 | ベルト投射上部MADモータ回転速度読み取り |
| encoder1 | ベルト投射上部MADモータ回転速度読み取り |
| limitSw0 | ばね投射装填読み取り |
| limitSw1 | ドリブル機構移動下限読み取り |
| limitSw2 | ベルト投射機構角度上限読み取り |
| limitSw3 | ベルト投射機構角度下限読み取り |
| LED | ロボット装飾用LEDテープ |
| MadMotor0 | ベルト投射上部MADモータ |
| MadMotor1 | ベルト投射下部MADモータ |
| MadMotor2 | ドリブル機構回転用MADモータ |
