# robot_controller

`joy_controller`から受け取った機構として意味のある操作指令を、各機構の制御判断に変換するパッケージです。CANフレームの組み立てや送受信は行わず、hardware_driverへROS topicとして指令をpublishします。

## カスタムメッセージ: `RobotCommand`

`robot_controller/msg/RobotCommand`は、`joy_controller`が`/robot_command`へpublishする操作指令です。各controller nodeは、必要なフィールドだけを使用します。

| フィールド | 型 | 用途 | 利用node |
| --- | --- | --- | --- |
| `is_intake` | `bool` | 吸気機構の操作状態 | 今後の吸気controller |
| `spring_is_fire` | `bool` | ばね射出の操作状態 | `spring_controller_node` |
| `belt_is_fire` | `bool` | ベルト射出の操作状態 | 今後のベルトcontroller |
| `belt_mode` | `uint8` | ベルトの動作モード | 今後のベルトcontroller |
| `dribble_mode` | `uint8` | ドリブルの動作モード | 今後のドリブルcontroller |
| `cmd_vel` | `geometry_msgs/Twist` | 機体の並進・角速度指令 | `mecanum_controller_node` |

`spring_controller_node`は、引き切り済みの`READY`状態で受けた`spring_is_fire`の`false → true`だけを発射要求として扱います。

## `mecanum_controller_node`

- node名: `mecanum_controller_node`
- 処理: `RobotCommand.cmd_vel`の並進・角速度から、4輪メカナムのホイール角速度を計算します。出力配列の順序は`[front_left, front_right, rear_left, rear_right]`です。

| 種別 | topic名 | 型 | 内容 |
| --- | --- | --- | --- |
| subscribe | `/robot_command` | `robot_controller/msg/RobotCommand` | `cmd_vel`から機体速度を受信 |
| publish | `/motor_vel` | `std_msgs/msg/Float32MultiArray` | 4輪のホイール角速度ベクトル `[rad/s]` |

主なパラメータは`wheel_radius`、`robot_length`、`robot_width`、`vx_sign`、`vy_sign`、`angular_z_sign`です。`robot_bringup/config/mecanum_controller.yaml`で設定します。

## `spring_controller_node`

- node名: `spring_controller_node`
- 処理: EduLite 05でばねを引き切り、発射後に再び装填する状態遷移を管理します。hardware_driverへは、CANではなくモータの速度指令だけをpublishします。

| 種別 | topic名（既定値） | 型 | 内容 |
| --- | --- | --- | --- |
| subscribe | `/robot_command` | `robot_controller/msg/RobotCommand` | `spring_is_fire`の発射操作を受信 |
| subscribe | `/limit_switches` | `std_msgs/msg/UInt8MultiArray` | リミットスイッチ配列。`data`は`std::vector<uint8_t>`として扱い、`0=false`、非0を`true`と判定 |
| publish | `/spring_edulite_speed` | `std_msgs/msg/Float32` | EduLite 05の目標速度 `[rad/s]` |

状態は`LOAD`、`READY`、`FIRE`です。

1. 起動時は`LOAD`で`loading_velocity_rad_s`をpublishし、ばねを引きます。
2. 設定した`limit_switch_index`がtrueになると`READY`へ遷移し、`0 rad/s`をpublishします。
3. `READY`中に限り、`spring_is_fire`の`false → true`を受けると`FIRE`へ遷移します。`LOAD`中の発射操作は無視します。
4. `FIRE`では`fire_velocity_rad_s`を`fire_duration_sec`の間publishし、完了後は`LOAD`に戻ります。

topic名、リミットスイッチのindex、各速度、発射時間は`robot_bringup/config/spring_controller.yaml`で設定できます。起動には`robot_bringup/launch/spring_controller.launch.py`を使います。
