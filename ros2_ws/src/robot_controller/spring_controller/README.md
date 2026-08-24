# spring_controller_node

ばね用EduLite 05を一方向のProfile Position制御で動かすノード。
起動時とEduLite再接続時にリミットスイッチで原点を取り、通常運転では
発射要求ごとに累積位置目標へ1周分の負角度を加算する。

## 関連ファイル

- 実装: `src/spring_edulite_controller.cpp`
- 宣言: `include/spring_controller/spring_edulite_controller.hpp`
- 設定: `robot_bringup/config/spring_controller.yaml`
- 起動: `robot_bringup/launch/controllers/spring_controller.launch.py`

## 入出力

| 種別 | topic/service | 型 | 内容 |
|---|---|---|---|
| sub | `/system/emergency_stop` | `Bool` | trueの間は目標値の更新を停止 |
| sub | `/spring/fire_request` | `Bool` | 立ち上がりごとに通常発射 |
| sub | `/spring/slow_fire_request` | `Bool` | 立ち上がりごとに低速発射 |
| sub | `/hardware/limit_switches` | `UInt8` | ホーミング用リミットスイッチ |
| sub | `/edulite/state` | `ActuatorState` | 接続状態と原点設定状態 |
| pub | `/edulite/target` | `ActuatorTarget` | logical ID 4の累積位置[rad] |
| pub | `/spring/actuator_ready` | `Bool` | 待機位置到達・発射可能フラグ |
| client | `/edulite/set_position` | `SetPosition` | リミット位置を0 radに設定 |

## 状態遷移

```text
起動・再接続
    │
    │ state.position_reference_set == false
    ▼
 HOMING ── limit ON / set_position(0)成功 ──→ MOVING_TO_STANDBY
    │                                             │
    └──────── timeout → ERROR                     │ 到達 & 静止
                                                  ▼
                                                READY ◄───────────────┐
                                                  │                   │
                                                  └ fire要求          │ 到達 & 静止
                                                    target += fire_increment
                                                    ▼                 │
                                                  FIRING ─────────────┘
```

### HOMING
EduLiteドライバは未原点中も`position_reference_set=false`を配信する。
上位ノードは`command_period_ms`ごとに負方向へゆっくり進む。リミット検出後、
上位ノードが`/edulite/set_position`を呼び、現在位置と累積目標を0 radへリセットする。

### MOVING_TO_STANDBY
0点設定後、目標位置を`standby_offset_rad`（正方向）に設定して待機位置へ逆回転移動する。
目標位置到達かつモーター静止を確認すると`READY`へ遷移する。

### READY
発射待機状態。`/spring/actuator_ready`に`true`を配信する。
発射要求のfalse→true立ち上がりごとに次を実行し、`FIRING`状態へ移行する。

```text
target_position_rad += fire_increment_rad
```

### FIRING
1周分（`fire_increment_rad`）回転して発射動作を行い、元の待機オフセット位置の位相へ戻る。
回転完了（目標位置到達かつ静止）を検知すると再び`READY`状態へ戻る。

### SLOW_FIRING_EXTENDING / SLOW_FIRING_RETURNING
低速発射では、指定速度で待機位置から押し出し位置まで進み、その後、指定した復帰速度で待機位置へ戻る。各区間は位置到達を連続して確認した場合だけ次の状態へ進む。想定時間を超えた場合は、現在のフィードバック位置をPP保持目標として送信し、`ERROR`へ遷移する。タイムアウトを正常完了として`READY`には戻さない。

## 再接続と非常停止

EduLiteがREADY以外になった場合、上位ノードは累積目標を0へ破棄する。
再びREADYになっても`position_reference_set=false`なら、必ずHOMINGから再開する。

非常停止中は累積目標値を変更しない。解除後も同じ目標位置を維持する。
非常停止中に届いた発射要求は拒否する。

## 主なパラメータ

| parameter | 内容 |
|---|---|
| `standby_offset_rad` | 0点合わせ後に待機位置へ移動するためのオフセット角度[rad] |
| `position_tolerance_rad` | 待機位置・目標位置到達判定の許容誤差[rad] |
| `fire_increment_rad` | 発射要求1回で加算する回転角度[rad] |
| `slow_fire_target_position_rad` | 低速発射のゼロ点基準の絶対目標位置[rad] |
| `slow_fire_base_velocity_rad_s` | 低速発射の押し出し速度[rad/s] |
| `slow_fire_return_velocity_rad_s` | 低速発射の復帰速度[rad/s] |
| `homing_velocity_rad_s` | ホーミング時の目標移動速度の大きさ[rad/s] |
| `homing_timeout_sec` | ホーミングのタイムアウト[s] |
| `stopped_velocity_threshold_rad_s` | 静止と判定する速度閾値[rad/s] |
| `required_stable_feedback_count` | 静止判定に必要な連続回数 |
| `command_period_ms` | 位置目標の更新・再送周期[ms] |
| `limit_switch_bit_offset` | リミットスイッチbyte内の使用bit |

## 確認方法

```bash
ros2 topic echo /edulite/state
ros2 topic echo /limit_switchs
ros2 topic echo /edulite/target
ros2 topic pub --once /spring/fire_request std_msgs/msg/Bool "{data: true}"
```

発射前に、logical ID 4の`position_reference_set`がtrueであることと、
機構周辺が安全であることを確認する。
