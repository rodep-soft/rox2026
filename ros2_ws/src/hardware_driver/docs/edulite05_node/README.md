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

CAN受信購読にはContent Filterを適用し、EduLiteのType 2フィードバック、
Type 17パラメーター応答、Type 21詳細故障フレームだけをコールバックへ渡す。

受信後も拡張データフレームかつDLC 8であることを確認し、RTR、CANエラーフレーム、
Type 17の宛先がホストIDではない応答は状態更新に使用しない。

Type 21の32bit詳細故障コード、またはType 2の故障要約が0以外になったモーターは`ERROR`へ移行し、保持していた目標値を破棄して通常指令を停止する。Type 21を受信した場合は、Type 2がfault継続を示している間、その詳細値を`fault_code`へ保持する。faultが消えてから200 ms経過すると再初期化を開始する。

`current_a`は`current_feedback_enabled=true`のモーターについてType 17で取得したフィルタ済みq軸電流`iqf` [A]である。無効時、取得前、または最後の正常応答から読出し周期の3倍を超えた場合はNaNになる。複数台の読出し要求は同一更新周期に集中させず、更新周期ごとに最大1台へ送信する。

## PP/CSPの位置基準

PP/CSPはモーターごとの`position_reference_mode`で位置基準の設定方法を選ぶ。

- `service`: driverを起動するたびに最初の有効な位置を一時原点として、ホーミング用の位置指令を許可する。この間も`position_reference_set=false`のため、上位ノードは通常動作を開始しない。`/edulite/set_position`が成功すると正式な位置基準になる。サービスの`position`には、呼び出し時点のモーター位置として扱いたい角度[rad]を指定する。
- `yaml_offset`: エンコーダの絶対角に`position_offset_rad`を加え、最初の有効な位置を受信した時点で正式な位置基準にする。

状態メッセージの`position_reference_set`は、serviceまたはYAMLによる正式な位置基準が確定した場合だけtrueになる。`TEMPORARY`な位置基準はホーミング指令にだけ使用し、正式な原点としては通知しない。サービスはPP/CSPかつフィードバック受信済みの場合だけ成功し、速度制御モーター、未知の`logical_id`、NaN/Infを拒否する。

```bash
ros2 service call /edulite/set_position actuator_msgs/srv/SetPosition \
  "{logical_id: 4, position: 0.0}"
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
| `current_feedback_enabled` | Type 17による電流取得を有効化 |
| `current_feedback_period_ms` | 電流読出し要求の最小周期 [ms] |
| `immediate_state_publish` | CAN更新時にこのモーターを`/edulite/state`へ即時配信する |
| `position_reference_mode` | PP/CSPの位置基準設定方法 |
| `position_offset_rad` | `yaml_offset`でエンコーダ絶対角に加える固定オフセット |
| `minimum_position_rad` | PP/CSP位置指令の下限 |
| `maximum_position_rad` | PP/CSP位置指令の上限 |

## 共通パラメーター

| パラメーター | 内容 |
|---|---|
| `update_period_ms` | 初期化・watchdog・CAN指令を確認する周期 [ms] |
| `state_array_publish_period_ms` | `/edulite/state_array`を配信する周期 [ms] |

初期化は`RUN_MODE`を設定・確認してからEnableし、続いて残りのパラメーターを設定する。PP/CSPではEnable時の意図しない移動を防ぐため、`RUN_MODE`確認後に0x7019から現在機械位置を読み、その値を`POSITION_REFERENCE`へ書き込んでReadback一致を確認してからEnableする。この保持位置は正式な原点には使用せず、位置基準状態はserviceまたはYAMLで確定する。応答待ちのモーターは送信対象から一時的に飛ばし、待機完了時は同じ更新周期内で次の読出しまたは再送フレームまで生成する。初期化フレームはCAN負荷を抑えるため1更新周期につき最大1つとする。

CANフレーム形式とEduLite固有パラメーターは`hardware_driver`内部に閉じ込め、上位ノードは`logical_id`、速度、位置だけを扱う。
