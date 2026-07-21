# 修正・実装TODO

## dribble実速度確認後のばね射出

- 現在は、dribble_controllerが減速指令を`0 rad/s`へ到達させたことを停止完了としている。
- STM32からdribbleモータの実速度を取得するtopicを追加し、実測速度が0付近であることを確認してから`spring_controller`が`FIRE`状態へ遷移するようにする。

## EduLiteドライバの集約

- `edulite_driver`はモータごとにnodeを起動せず、1 nodeで全EduLiteモータを処理する。
- controller側はモータIDやCAN仕様を持たず、機構として意味のあるtopicをpublishする。
- `edulite_driver`側のconfig.yamlで、topic、motor ID、制御種別（position / velocity）、正逆転の符号を管理する。

### controllerからedulite_driverへ渡すtopic案

| 機構 | topic | 型 | 内容 |
| --- | --- | --- | --- |
| mecanum | `/mecanum_wheel_velocity_command` | `std_msgs/msg/Float32MultiArray` | ホイール角速度ベクトル `[rad/s]`。順序は`[front_left, front_right, rear_left, rear_right]`で固定 |
| spring | `/spring_velocity_command` | `std_msgs/msg/Float32` | ばねを引き切るベルト用EduLiteの目標角速度 `[rad/s]` |
| dribble | `/dribble/rpm` | `std_msgs/msg/Int16` | dribble用の目標回転数 `[RPM]` |

### edulite_driverの設定案

```yaml
mecanum:
  topic: /mecanum_wheel_velocity_command
  motor_ids: [1, 2, 3, 4]
  control_type: velocity
  velocity_sign: [1.0, 1.0, -1.0, -1.0]

spring:
  topic: /spring_velocity_command
  motor_id: 5
  control_type: velocity
  velocity_sign: 1.0

dribble:
  topic: /dribble/rpm
  motor_id: 6
  control_type: velocity
  velocity_sign: -1.0
```

- position / velocityを実行中に切り替える必要が出た場合だけ、制御modeを含む専用カスタムメッセージを検討する。

## ばね射出時の駆動方式

- ばね用RobStrideは、`LOAD`中に`loading_velocity_rad_s`を出してばねを引く。
- `READY`では`0 rad/s`で待機する。
- `FIRE`では`fire_velocity_rad_s`を`fire_duration_sec`の間出して、RobStrideを回転させて発射する。
- 逆回転で発射する場合、`loading_velocity_rad_s`と`fire_velocity_rad_s`は逆符号に設定する。実機に合わせてYAMLで校正する。

## beltの動作モード

- 各速度モードの実際の目標角速度、motor ID、正逆転は`edulite_driver`側config.yamlで管理する。
