# edulite05_node

EduLite 05を1ノードで複数台管理し、速度制御、Profile Position（PP）制御、Cyclic Synchronous Position（CSP）制御を提供する。

## ROSインターフェース

| 種別 | 既定名 | 型 | 用途 |
|---|---|---|---|
| subscribe | `/edulite/target` | `actuator_msgs/msg/ActuatorTarget` | 1台の速度または位置指令 |
| subscribe | `/edulite/target_array` | `actuator_msgs/msg/ActuatorTargetArray` | 複数台の速度または位置指令 |
| publish | `/edulite/state` | `actuator_msgs/msg/ActuatorState` | CAN受信時に更新された1台の状態 |
| publish | `/edulite/state_array` | `actuator_msgs/msg/ActuatorStateArray` | 全モーターの周期状態 |
| service | `/edulite/set_position` | `actuator_msgs/srv/SetPosition` | PP/CSPの現在位置を指定角度として校正 |

速度指令の単位はrad/s、位置指令と位置状態の単位はradである。

## PP/CSPの位置基準

PP/CSPは位置基準が設定されるまで位置指令をCANへ送らない。モーターごとの
`position_reference_mode`で設定方法を選ぶ。

- `service`: `/edulite/set_position`が成功するまで待つ。サービスの`position`には、呼び出し時点のモーター位置として扱いたい角度[rad]を指定する。0.0なら現在位置がゼロ点になる。
- `yaml_absolute`: 最初に受信したフィードバック位置を`startup_absolute_position_rad`として扱い、自動的にオフセットを計算する。

状態メッセージの`position_reference_set`で位置指令が許可されたか確認できる。サービスはPP/CSPかつフィードバック受信済みの場合だけ成功し、速度制御モーター、未知の`logical_id`、NaN/Infを拒否する。

```bash
ros2 service call /edulite/set_position actuator_msgs/srv/SetPosition \
  "{logical_id: 5, position: 0.0}"
```

## モーター別パラメーター

| パラメーター | 内容 |
|---|---|
| `logical_id` | ROS側で使用する一意なID |
| `can_id` | EduLite 05のCAN ID |
| `control_mode` | `velocity`、`pp`、`csp` |
| `current_limit` | 電流制限 |
| `acceleration` | 加速度。velocityとPPで使用 |
| `speed_limit` | 速度制限。velocity、PP、CSPで使用 |
| `command_period_ms` | CAN指令の最小送信周期 |
| `target_timeout_ms` | velocity指令が途絶えた際に0へ戻す時間 |
| `feedback_timeout_ms` | 接続切れと判定する時間 |
| `position_reference_mode` | PP/CSPの位置基準設定方法 |
| `startup_absolute_position_rad` | `yaml_absolute`起動時の現在角 |
| `minimum_position_rad` | PP/CSP位置指令の下限 |
| `maximum_position_rad` | PP/CSP位置指令の上限 |

CANフレーム形式とEduLite固有パラメーターは`hardware_driver`内部に閉じ込め、上位ノードは`logical_id`、速度、位置だけを扱う。
