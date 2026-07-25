## Joyボタン・軸の配置

`joy_controller`は`joy_node`からの`/joy`を受け、機構ごとの指令topicへ変換します。既定では`robot_bringup/launch/joy_controller.launch.py`が`/dev/input/event0`を読みます。

### Buttons

`sensor_msgs/msg/Joy`の`buttons[]`に対応する。

| Index | ボタン |
|---:|---|
| 0 | × |
| 1 | ○ |
| 2 | □ |
| 3 | △ |
| 4 | Create |
| 5 | PS |
| 6 | Options |
| 7 | L3 |
| 8 | R3 |
| 9 | L1 |
| 10 | R1 |
| 11 | 方向キー 上 |
| 12 | 方向キー 下 |
| 13 | 方向キー 左 |
| 14 | 方向キー 右 |
| 15 | 中央ボタン（Home） |

### Axes

`sensor_msgs/msg/Joy`の`axes[]`に対応する。

| Index | 入力 |
|---:|---|
| 0 | 左スティック 左右（左=1、右=-1） |
| 1 | 左スティック 上下（上=1、下=-1） |
| 2 | 右スティック 左右（左=1、右=-1） |
| 3 | 右スティック 上下（上=1、下=-1） |
| 4 | L2（通常=1、押下=-1） |
| 5 | R2（通常=1、押下=-1） |

### 操作仕様

| 操作 | 入力 | 動作 |
|---|---|---|
| 最大開放 | L2 + Options | `MAX_OPEN (-1.3 rad)`へ移動。もう一度でDRIBBLEへ復帰 |
| 非常停止 | Create + Home | 停止状態をON/OFF。ON時はDRIBBLE位置へ復帰 |
| ベルトmode | DPAD 上 / 下 | modeを1段階増減 |
| ドリブル | R2 + Home | ON/OFFを切替 |
| Spring | L1 + ○ | ON/OFFを切替。最大開放中は出力しない |
| 前後反転 | PS | `linear.x`のみ符号を反転 |
| 取り込み・投射 | L2 + ○ | 両ベルトが目標RPMへ到達後、`INTAKE → SHOOT → DRIBBLE`を一度だけ要求 |

L2 + ○を押した時点でベルトが未到達なら要求を保留し、`/belt/ready`が`true`になった時点で実行します。位置移動中に同じ要求を送っても、`dribble_position_controller`が無視します。

### Publish / subscribe topic

| 種別 | topic名 | 型 | 内容 |
|---|---|---|---|
| publish | `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` | 機体速度指令 |
| publish | `/spring/fire_request` | `std_msgs/msg/Bool` | Spring ON/OFF |
| publish | `/belt/mode` | `std_msgs/msg/UInt8` | ベルト速度mode |
| publish | `/dribble/enabled` | `std_msgs/msg/Bool` | ドリブルON/OFF |
| publish | `/dribble/position_mode` | `std_msgs/msg/UInt8` | `0=DRIBBLE`、`1=SHOOT`、`2=MAX_OPEN` |
| publish | `/dribble/intake_shoot_request` | `std_msgs/msg/Bool` | `true`で位置シーケンスを開始 |
| publish | `/emergency_stop` | `std_msgs/msg/Bool` | 非常停止状態 |
| subscribe | `/belt/ready` | `std_msgs/msg/Bool` | 両ベルトが目標RPMに到達した状態 |

### 確認コマンド

```bash
ros2 topic echo /joy
```
