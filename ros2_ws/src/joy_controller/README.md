## Joyボタン・軸の配置

`joy_controller`は`joy_node`からの`/joy`を受け、機構ごとの指令topicへ変換します。既定では`robot_bringup/launch/joy_controller.launch.py`が`/dev/input/js0`を読みます。

### Buttons

`sensor_msgs/msg/Joy`の`buttons[]`に対応する。

| Index | ボタン |
|---:|---|
| 0 | □（Square） |
| 1 | ×（Cross） |
| 2 | ○（Circle） |
| 3 | △（Triangle） |
| 4 | L1 |
| 5 | R1 |
| 6 | L2 |
| 7 | R2 |
| 8 | Create / Share |
| 9 | Options |
| 10 | L3（左スティック押し込み） |
| 11 | R3（右スティック押し込み） |
| 12 | PS |
| 13 | タッチパッド（Home） |

### Axes

`sensor_msgs/msg/Joy`の`axes[]`に対応する。

| Index | 入力 |
|---:|---|
| 0 | 左スティック 左右 |
| 1 | 左スティック 上下 |
| 2 | 右スティック 左右 |
| 3 | L2（通常=`1`、押下=`-1`） |
| 4 | R2（通常=`1`、押下=`-1`） |
| 5 | 右スティック 上下 |
| 6 | 十字キー 左右 |
| 7 | 十字キー 上下 |

### 操作仕様

| 操作 | 入力 | 動作 |
|---|---|---|
| 最大開放 | L2 + Options | `MAX_OPEN (-1.3 rad)`へ移動。もう一度でDRIBBLEへ復帰 |
| 非常停止 | Create + Home | 停止状態をON/OFF。ON時はDRIBBLE位置へ復帰 |
| ベルトmode | DPAD 上 / 下 | modeを1段階増減 |
| ドリブル | R1 | ON |
| ドリブル | R1 + Home | OFF |
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
