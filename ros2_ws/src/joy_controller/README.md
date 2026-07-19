## Joyボタン・軸の配置

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
| 12 | PSボタン |
| 13 | タッチパッド押し込み |

### Axes

`sensor_msgs/msg/Joy`の`axes[]`に対応する。

| Index | 入力 |
|---:|---|
| 0 | 左スティック 左右 |
| 1 | 左スティック 上下 |
| 2 | 右スティック 左右 |
| 3 | L2 |
| 4 | R2 |
| 5 | 右スティック 上下 |
| 6 | 十字キー 左右 |
| 7 | 十字キー 上下 |

### 確認コマンド

```bash
ros2 topic echo /joy
```

## ボタン・軸の設定

`robot_bringup/config/joy.yaml` で、各機構のボタンと各軸の番号を変更できる。
設定値の名前は cpp 側で対応する押下状態の名前と共通で、値はボタンまたは軸の番号を表す。

## 操作仕様

既定のボタン番号では、以下の操作を行う。

| 操作 | 入力 |
|---|---|
| 吸気 ON/OFF | L2 + △ |
| ばね射出 ON/OFF | R2 + ○ |
| ベルト射出 ON/OFF | L2 + ○ |
| ベルト mode 増減 | L2 + DPAD 上下 |
| ドリブル mode 増減 | R2 + DPAD 上下 |
| 緊急停止 | Create + タッチパッド |

ベルトmodeは`STOP (0)`、`LEVEL_1 (1)`、`LEVEL_2 (2)`、`LEVEL_3 (3)`、
`LEVEL_4 (4)`の5状態、dribble modeは`STOP (0)`、`LOW (1)`、`HIGH (2)`の3状態。
DPAD上で1段階増加、DPAD下で1段階減少し、範囲外には遷移しない。

`*_is_enable_button` は機構操作を有効化するボタン、`*_button_on` は実行する
ボタンの番号を表す。これらは名前に `is` や `on` を含むが、YAML では bool ではなく
ボタン番号を設定する。

enable ボタンの押下だけでは操作を実行しない。△、○、タッチパッド、および DPAD は
`off -> on` になった瞬間だけ受け付ける。L2 と R2 を同時に押した場合も、両方の機構を
有効化する。

## 速度指令

| Joy 軸 | 出力 |
|---|---|
| 左スティック y | `cmd_vel.linear.x` |
| 左スティック x | `cmd_vel.linear.y` |
| 右スティック x | `cmd_vel.angular.z` |

スティック入力は、絶対値が `axis_deadzone` 未満の場合に `0.0` として扱う。
既定値は `0.05`。DPAD のような ON/OFF 判定に使う軸は、絶対値が
`axis_on_threshold` 以上で ON として扱う。既定値は `0.7`。

速度指令は deadzone 適用後の軸入力に対して、正方向には `max_*`、負方向には
`min_*` を掛け、さらに `linear_x_scale`、`linear_y_scale`、`angular_z_scale` を掛ける。
既定の速度範囲は `linear.x` / `linear.y` が ±2.0 m/s、`angular.z` が ±2.0 rad/s。

## 非常停止

Create + タッチパッドを押している間は、通常の操作より先に非常停止として処理する。
`RobotCommand`の全フィールドをゼロ値でpublishし、非常停止serviceは押下の立上り時に1回だけ呼び出す。
非常停止中も入力の前回値を更新するため、解除時に押しっぱなしのボタンやDPADを新規操作として扱わない。
