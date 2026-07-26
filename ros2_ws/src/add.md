# 4 mode統合controller仕様

## operation mode

`/operation_mode`は`std_msgs/msg/UInt8`でpublishする。

| 値 | mode | 内容 |
|---:|---|---|
| 0 | STOP | 全機構を停止し、positionをDRIBBLE位置へ戻す |
| 1 | DRIVE | 通常走行・通常機構操作 |
| 2 | INTAKE_AND_SHOOT | RPM到達確認後にINTAKE→SHOOT→DRIBBLEを実行できる |
| 3 | GAME2_MODE | belt以外の機構を制限し、positionをMAX_OPEN位置へ移動する |

## mode遷移

```text
STOP -- L2 + Options --> DRIVE

DRIVE <--------- L2 + Options ---------> INTAKE_AND_SHOOT

DRIVE <------ L2 + R2 + Options ------> GAME2_MODE

DRIVE / INTAKE_AND_SHOOT / GAME2_MODE -- L2 + Home --> STOP
```

- INTAKE_AND_SHOOTの位置シーケンス実行中は、STOP以外のmode変更を無視する。
- positionがDRIBBLE位置へ到達したら`/operation_mode_complete=true`を送り、JoyがDRIVEへ戻す。
- 非常停止はoperation modeと分け、すべてのmodeより優先する。

## modeごとの操作

| 操作 | STOP | DRIVE | INTAKE_AND_SHOOT | GAME2_MODE |
|---|---:|---:|---:|---:|
| belt DPAD上下 | 停止 | 可 | 可 | 可 |
| dribble R1 ON / R1+Home OFF | 停止 | 可 | 可 | targetを0 RPMへ強制 |
| Spring L1+○ | 禁止 | 可 | 禁止 | 禁止 |
| R2+左: SHOOT位置 | 禁止 | 可 | 可 | 禁止 |
| R2+右: MAX_OPEN位置 | 禁止 | 可 | 可 | 禁止 |
| L2+○: INTAKE→SHOOT→DRIBBLE | 禁止 | 禁止 | RPM到達時のみ可 | 禁止 |
| PS: 前後反転 | 可 | 可 | 可 | 可 |
| mecanum並進 | 停止 | 可 | 禁止 | 禁止 |
| mecanum旋回 | 停止 | 可 | 可 | 可 |

## belt・dribble統合controller

- node名は`belt_dribble_controller_node`とする。
- configは`belt_dribble_controller.yaml`へ統合する。
- beltとdribbleの目標RPMを周期送信する。
- underbelt、upperbelt、dribbleの実RPM feedbackを読む。
- INTAKE_AND_SHOOT中だけ3モータのRPM到達状態を判定する。
- RPM未到達時のL2+○は予約せず無視する。
- 3モータの実RPMと目標RPMが許容範囲内に一定時間入った状態でL2+○を受けると、
  `/intake_and_shoot=true`を一度だけpublishする。
- GAME2_MODE中はbeltを通常制御し、dribble targetを0 RPMへ強制する。

## Spring・position統合controller

- node名は`spring_position_controller_node`とする。
- configは`spring_position_controller.yaml`へ統合する。
- DRIVE以外ではSpringの発射要求を受け付けない。
- `/intake_and_shoot=true`を受けたらposition feedbackを確認しながら
  `INTAKE → SHOOT → DRIBBLE`を実行する。
- GAME2_MODEへ入ったらMAX_OPEN位置へ移動し、解除まで他のposition指令を受け付けない。
- STOPへ入ったらDRIBBLE位置へ戻す。
- 位置移動中は同じ目標radを周期送信し、許容範囲内のfeedbackを受けて次へ進む。

## mecanum controller

- `/operation_mode`をsubscribeする。
- STOPでは並進・旋回をともに0にする。
- DRIVEでは通常の速度指令を通す。
- INTAKE_AND_SHOOTとGAME2_MODEでは`linear.x/y`を0にし、`angular.z`だけ通す。

## topic

| 向き | topic | 型 | 内容 |
|---|---|---|---|
| Joy → controllers | `/operation_mode` | `std_msgs/msg/UInt8` | 現在の4 mode |
| Joy → belt/dribble | `/belt/mode` | `std_msgs/msg/UInt8` | belt速度mode |
| Joy → belt/dribble | `/dribble/enabled` | `std_msgs/msg/Bool` | dribble通常ON/OFF |
| Joy → belt/dribble | `/game2_command` | `std_msgs/msg/Bool` | L2+○による実行要求 |
| Joy → Spring/position | `/spring/fire_request` | `std_msgs/msg/Bool` | Spring発射要求 |
| Joy → Spring/position | `/dribble/position_mode` | `std_msgs/msg/UInt8` | 手動位置指令 |
| belt/dribble → Spring/position | `/intake_and_shoot` | `std_msgs/msg/Bool` | RPM到達後の位置シーケンス開始 |
| Spring/position → Joy | `/position_sequence_active` | `std_msgs/msg/Bool` | positionが実際にシーケンスを開始した状態 |
| belt/dribble → monitoring | `/shoot_ready` | `std_msgs/msg/Bool` | 3モータのRPM到達状態 |
| Spring/position → Joy | `/operation_mode_complete` | `std_msgs/msg/Bool` | 位置シーケンス完了 |
| hardware → belt/dribble | `*/current/rpm` | `std_msgs/msg/Int16` | 3モータの実RPM |
| belt/dribble → hardware | `*/target/rpm` | `std_msgs/msg/Int16` | 3モータの目標RPM |
| hardware → Spring/position | `/dribble/position_feedback` | `std_msgs/msg/Float32` | position実位置[rad] |
