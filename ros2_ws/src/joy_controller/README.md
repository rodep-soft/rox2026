# joy_controller

`sensor_msgs/msg/Joy`を機構指令と4つのoperation modeへ変換する。

## Joy配置

### buttons

| index | ボタン |
|---:|---|
| 0 | □ |
| 1 | × |
| 2 | ○ |
| 3 | △ |
| 4 | L1 |
| 5 | R1 |
| 6 | L2 |
| 7 | R2 |
| 8 | Create |
| 9 | Options |
| 10 | L3 |
| 11 | R3 |
| 12 | PS |
| 13 | タッチパッド（Home） |

### axes

| index | 入力 |
|---:|---|
| 0 | 左スティック左右 |
| 1 | 左スティック上下 |
| 2 | 右スティック左右 |
| 3 | L2（通常=`1`、押下=`-1`） |
| 4 | R2（通常=`1`、押下=`-1`） |
| 5 | 右スティック上下 |
| 6 | DPAD左右（左=`1`、右=`-1`） |
| 7 | DPAD上下（上=`1`、下=`-1`） |

## operation mode

| 値 | mode |
|---:|---|
| 0 | STOP |
| 1 | DRIVE |
| 2 | INTAKE_AND_SHOOT |
| 3 | GAME2_MODE |

```text
STOP -- L2 + Options --> DRIVE
DRIVE <------ L2 + Options ------> INTAKE_AND_SHOOT
DRIVE <-- L2 + R2 + Options ----> GAME2_MODE
STOP以外 -- L2 + Home ----------> STOP
```

INTAKE_AND_SHOOTの位置シーケンス実行中は、STOP以外のmode変更を無視する。
`/operation_mode_complete=true`を受けるとDRIVEへ戻る。

## 操作

| 入力 | 動作 |
|---|---|
| DPAD上 / 下 | belt modeを増減 |
| R1 | dribble ON |
| R1 + Home | dribble OFF |
| L1 + ○ | Spring ON/OFF。DRIVEだけで受理 |
| L2 + ○ | INTAKE_AND_SHOOT中の実行要求 |
| R2 + DPAD左 | SHOOT位置へ移動。DRIVE・INTAKE_AND_SHOOTだけで受理 |
| R2 + DPAD右 | MAX_OPEN位置へ移動。DRIVE・INTAKE_AND_SHOOTだけで受理 |
| PS | `linear.x`の前後反転 |
| Create + Home | 非常停止ON/OFF |

## 主なtopic

| 種別 | topic | 型 |
|---|---|---|
| publish | `/operation_mode` | `std_msgs/msg/UInt8` |
| publish | `/game2_command` | `std_msgs/msg/Bool` |
| publish | `/belt/mode` | `std_msgs/msg/UInt8` |
| publish | `/dribble/enabled` | `std_msgs/msg/Bool` |
| publish | `/spring/fire_request` | `std_msgs/msg/Bool` |
| publish | `/dribble/position_mode` | `std_msgs/msg/UInt8` |
| publish | `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` |
| publish | `/emergency_stop` | `std_msgs/msg/Bool` |
| subscribe | `/intake_and_shoot` | `std_msgs/msg/Bool` |
| subscribe | `/operation_mode_complete` | `std_msgs/msg/Bool` |
