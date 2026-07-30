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
| 2 | SHOT_CYCLE |
| 3 | BELT_ONLY |

```text
STOP -- Home --> DRIVE
STOP・DRIVE <-- Create --> SHOT_CYCLE
STOP・DRIVE <-- Options --> BELT_ONLY
STOP以外 -- Home --> STOP
```

SHOT_CYCLEの動作中でもHomeはSTOPとして受理する。
SHOT_CYCLEの動作中は、Home以外のmode変更を無視する。
BELT_ONLY中のCreateと、SHOT_CYCLE中のOptionsは無視する。
`auto_drive_on_shot_cycle_complete=true`の場合、
`/shot_cycle/complete=true`を受けるとDRIVEへ戻る。

## 操作

| 入力 | 動作 |
|---|---|
| Home | STOPへ移動。STOP中はDRIVEへ戻る |
| Create | STOP・DRIVEからSHOT_CYCLEへ移動。同modeの待機中はDRIVEへ戻る |
| Options | STOP・DRIVEからBELT_ONLYへ移動。同mode中はDRIVEへ戻る |
| DPAD上 / 下 | belt modeをSTOPとLEVEL_1〜LEVEL_6の範囲で増減 |
| R1 | dribble ON/OFF。DRIVE・SHOT_CYCLEだけで受理 |
| L1 + ○ | Spring発射要求。DRIVEだけで、押下中にtrueを送る |
| L2 + ○ | SHOT_CYCLE中の実行要求 |
| R2 + DPAD左 | DRIBBLE位置へ移動。DRIVE・SHOT_CYCLEだけで受理 |
| R2 + DPAD右 | OPEN位置へ移動。DRIVE・SHOT_CYCLEだけで受理 |
| PS | `linear.x/y`の前後左右反転 |

STOPへ入った場合とJoy通信が途切れた場合は、belt mode・dribble状態・
位置シーケンス状態を保持せず初期化する。
Joy通信が途切れた場合はSTOPへ移動し、Joy入力が復帰するまで状態の周期publishを止める。

## 主なparameter

| parameter | 型 | 説明 |
|---|---|---|
| `joy_timeout_ms` | `int` | Joy入力断でSTOPへ移るまでの時間[ms] |
| `state_publish_period_ms` | `int` | belt mode、dribble enabled、operation mode、spring fire requestの再送周期[ms] |
| `auto_drive_on_shot_cycle_complete` | `bool` | `true`ならshot cycle完了時にDRIVEへ自動復帰する |

### belt mode

| 値 | mode |
|---:|---|
| 0 | STOP |
| 1 | LEVEL_1 |
| 2 | LEVEL_2 |
| 3 | LEVEL_3 |
| 4 | LEVEL_4 |
| 5 | LEVEL_5 |
| 6 | LEVEL_6 |

### position

| 値 | position |
|---:|---|
| 0 | DRIBBLE |
| 1 | INTAKE |
| 2 | SHOT |
| 3 | OPEN |

## 主なtopic

| 種別 | topic | 型 |
|---|---|---|
| publish | `/operation_mode` | `std_msgs/msg/UInt8` |
| publish | `/shot_cycle/request` | `std_msgs/msg/Bool` |
| publish | `/belt/mode` | `std_msgs/msg/UInt8` |
| publish | `/dribble/enabled` | `std_msgs/msg/Bool` |
| publish | `/spring/fire_request` | `std_msgs/msg/Bool` |
| publish | `/dribble/position_mode` | `std_msgs/msg/UInt8` |
| publish | `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` |
| publish | `/emergency_stop` | `std_msgs/msg/Bool` |
| subscribe | `/shot_cycle/running` | `std_msgs/msg/Bool` |
| subscribe | `/shot_cycle/complete` | `std_msgs/msg/Bool` |
