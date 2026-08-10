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
| sub | `/emergency_stop` | `Bool` | trueの間は目標値の更新を停止 |
| sub | `/spring/fire_request` | `Bool` | 立ち上がりごとに1周分を加算 |
| sub | `/limit_switchs` | `UInt8` | ホーミング用リミットスイッチ |
| sub | `/edulite/state` | `ActuatorState` | 接続状態と原点設定状態 |
| pub | `/edulite/target` | `ActuatorTarget` | logical ID 4の累積位置[rad] |
| client | `/edulite/set_position` | `SetPosition` | リミット位置を0 radに設定 |

## 状態遷移

```text
起動・再接続
    │
    │ state.position_reference_set == false
    ▼
 HOMING ── limit ON / set_position(0)成功 ──→ READY
    │                                         │
    └──────── timeout → ERROR                 └ fire要求
                                                  target += fire_increment_rad
```

### HOMING

EduLiteドライバは未原点中も`position_reference_set=false`を配信する。
上位ノードは`command_period_ms`ごとに次の差分を累積目標へ加える。

```text
-homing_velocity_rad_s * command_period_ms / 1000
```

したがってホーミング指令は負方向へゆっくり進む。リミット検出後、
上位ノードが`/edulite/set_position`を呼び、現在位置と累積目標を0 radへ戻す。

### READY

発射要求のfalse→true立ち上がりごとに次だけを実行する。

```text
target_position_rad += fire_increment_rad
```

`fire_increment_rad`は必ず負値とする。装填・発射タイマーや逆方向への復帰処理はない。
リミットスイッチは通常発射中には使用しない。

## 再接続と非常停止

EduLiteがREADY以外になった場合、上位ノードは累積目標を0へ破棄する。
再びREADYになっても`position_reference_set=false`なら、必ずHOMINGから再開する。

非常停止中は累積目標値を変更しない。解除後も同じ目標位置を維持する。
非常停止中に届いた発射要求は拒否する。

## 主なパラメータ

| parameter | 内容 |
|---|---|
| `fire_increment_rad` | 発射要求1回で加算する負角度[rad] |
| `homing_velocity_rad_s` | ホーミング時の目標移動速度の大きさ[rad/s] |
| `homing_timeout_sec` | ホーミングのタイムアウト[s] |
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
