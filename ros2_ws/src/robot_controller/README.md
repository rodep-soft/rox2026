# robot_controller

Joyから受けた機構指令と`/operation_mode`を制御判断へ変換し、hardware_driverへ
機構として意味のある目標値をpublishする。CAN ID、フレーム形式、エンディアンは
hardware_driverが担当する。

## node構成

```text
joy_controller
  ├─ operation_mode ──────────┬─ belt_dribble_controller
  │                           ├─ spring_position_controller
  │                           └─ mecanum_controller
  ├─ belt / dribble指令 ─────── belt_dribble_controller
  ├─ Spring / position指令 ──── spring_position_controller
  └─ cmd_vel ───────────────── mecanum_controller
```

| node | 主な責務 | config |
|---|---|---|
| `belt_dribble_controller_node` | belt・dribbleの目標RPM送信、3実RPMの到達判定 | `belt_dribble_controller.yaml` |
| `spring_position_controller_node` | Spring状態遷移、position feedbackによる位置遷移 | `spring_position_controller.yaml` |
| `mecanum_controller_node` | mode制限と4輪メカナム逆運動学 | `mecanum_controller.yaml` |

## operation mode

| 値 | mode | belt・dribble | Spring・position | mecanum |
|---:|---|---|---|---|
| 0 | STOP | 全targetを0 RPM | Spring停止、DRIBBLE位置へ復帰 | 全輪停止 |
| 1 | DRIVE | 通常制御 | Spring・手動position可 | 通常走行 |
| 2 | INTAKE_AND_SHOOT | RPM到達判定 | Spring禁止、位置シーケンス可 | 旋回のみ |
| 3 | GAME2_MODE | belt通常、dribble 0 RPM | Spring禁止、MAX_OPENへ移動 | 旋回のみ |

非常停止はoperation modeより優先する。

## belt_dribble_controller_node

### subscribe

| topic | 型 | 内容 |
|---|---|---|
| `/operation_mode` | `std_msgs/msg/UInt8` | 現在のoperation mode |
| `/belt/mode` | `std_msgs/msg/UInt8` | belt速度mode |
| `/dribble/enabled` | `std_msgs/msg/Bool` | dribble通常ON/OFF |
| `/game2_command` | `std_msgs/msg/Bool` | L2+○による実行要求 |
| `/underbelt/current/rpm` | `std_msgs/msg/Int16` | underbelt実RPM |
| `/upperbelt/current/rpm` | `std_msgs/msg/Int16` | upperbelt実RPM |
| `/dribble/current/rpm` | `std_msgs/msg/Int16` | dribble実RPM |
| `/emergency_stop` | `std_msgs/msg/Bool` | 非常停止 |

### publish

| topic | 型 | 内容 |
|---|---|---|
| `/underbelt/target/rpm` | `std_msgs/msg/Int16` | underbelt目標RPM |
| `/upperbelt/target/rpm` | `std_msgs/msg/Int16` | upperbelt目標RPM |
| `/dribble/target/rpm` | `std_msgs/msg/Int16` | dribble目標RPM |
| `/shoot_ready` | `std_msgs/msg/Bool` | 3モータのRPM到達状態 |
| `/intake_and_shoot` | `std_msgs/msg/Bool` | 位置シーケンス開始 |

INTAKE_AND_SHOOT中に3実RPMが各targetの許容範囲内へ入り、
`ready_hold_sec`継続すると`/shoot_ready=true`になる。この状態で
`/game2_command=true`を受けた場合だけ`/intake_and_shoot=true`を一度publishする。
未到達時の要求は予約せず無視する。

## spring_position_controller_node

Springは`LOAD`、`READY`、`FIRE`の状態を持つ。発射要求はDRIVEだけで受け付ける。
STOP、INTAKE_AND_SHOOT、GAME2_MODE、非常停止中は`0 rad/s`を維持する。

positionは移動中、`position_command_period_ms`周期で同じ目標radを送り続け、
`/dribble/position_feedback`が許容範囲内へ入ったことを確認して次へ進む。

```text
/intake_and_shoot=true
  → INTAKE
  → feedback到達
  → SHOOT
  → feedback到達・保持
  → DRIBBLE
  → feedback到達
  → /operation_mode_complete=true
```

手動position指令はDRIVEとINTAKE_AND_SHOOTの待機中だけ受け付ける。

| `/dribble/position_mode` | 位置 |
|---:|---|
| 0 | DRIBBLE |
| 1 | SHOOT |
| 2 | MAX_OPEN |

GAME2_MODEへ入ると自動でMAX_OPENへ移動し、DRIVEへ戻るとDRIBBLEへ復帰する。
STOPと非常停止でもDRIBBLE位置へ復帰する。

## mecanum_controller_node

`/mecanum/cmd_vel`を4輪の角速度へ変換する。

- STOPまたは非常停止: `linear.x/y`と`angular.z`をすべて0にする。
- DRIVE: 全速度成分を通す。
- INTAKE_AND_SHOOT・GAME2_MODE: `linear.x/y`を0にし、`angular.z`だけ通す。

mode変更callbackでも直前のcmd_velから再計算するため、次のJoy入力を待たず制限を反映する。

## 起動

全controllerとhardware、Joyは以下で起動する。

```bash
ros2 launch robot_bringup robot.launch.py
```
