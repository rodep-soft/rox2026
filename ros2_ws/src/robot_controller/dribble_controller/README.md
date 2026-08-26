# dribble_controller

ドリブル機構のローラー回転と姿勢角度を1ノードで制御する。

## 入出力

| 種別 | topic | 内容 |
|---|---|---|
| sub | `/dribble/command_enabled` | ローラー正転ON/OFF |
| sub | `/dribble/position_mode` | `DRIBBLE``OPEN``FEED` 姿勢 |
| sub | `/dribble/command_position` | `DRIBBLE``OPEN``FEED``CUSTOM` 姿勢 |
| sub | `/dribble/shot_cycle_request` | `FEED → DRIBBLE` 自動射出動作 |
| sub | `/belt/command_mode` | beltの現在モード把握（0=STOP1〜4=LEVEL） |
| sub | `/system/emergency_stop` | ローラー停止安全姿勢復帰 |
| pub | `/vesc/target` | ローラー目標RPM |
| pub | `/edulite/target` | 姿勢目標角度[rad] |
| pub | `/belt/command_mode` | shot cycle時のbelt自動ON/OFF |
| pub | `/dribble/shot_cycle_state` | サイクル進行フェーズ（LED通知用） |

## 各姿勢（Position Mode）の役割
- `OPEN`：ボール排出用姿勢（角度: `-1.27 rad`回転: `0 RPM`）。
- `FEED`：ベルト射出押し込み姿勢（角度: `0.5 rad`）。
- `CUSTOM`：手動操作用の任意姿勢（角度: `custom_position_rad`）。

## shot cycleの動作

`/shot_cycle/request`にtrueが届くとOPEN状態を経由せずに直接射出角度(FEED)へ移行してボールを射出する。

### beltがSTOPの場合（自動spin-up）

1. ボタン押下と同時にドリブルローラを shot_cycle_opening_rpm へ上げ、beltを自動ONする。
2. 同時に、ばねを現在の累積待機位置から standby_offset_rad 分だけ収納する。
3. belt_shot_delay_sec の経過と、ばねの収納・停止完了の両方を待つ。
4. DRIBBLEからFEEDへ移動して押し込み射出する。
5. DRIBBLEへ戻り、ばねを累積待機位置へ戻して、自動起動したbeltを停止する。

### beltが既に回っている場合

beltはそのまま維持する。ドリブルローラの高回転化とばね収納を同時に開始し、
belt_shot_delay_sec の経過とばね収納完了後にFEEDへ移動する。

手動でアーム位置を変更すると shot cycle は中断される。
emergency stopが有効な場合はshot cycle要求を無視する。

## 実行中に変更できるparameter

- `dribble_on_rpm`（DRIBBLE姿勢・ボール保持中のRPM）
- `slow_fire_dribble_rpm`（スロー発射中のRPM。負値で逆回転）
- `ball_detection_threshold_a`（ボール検知閾値[A]デフォルト1.7）
- `ball_lost_threshold_a`（ボール喪失閾値[A]デフォルト1.0）
- `current_lpf_alpha`（電流値一次ローパスフィルタ最新値係数デフォルト0.3）
- `cmd_vel_timeout_sec`（IMU補正後速度指令が途絶えたと判定する時間）
- `cmd_vel_acceleration_lpf_alpha`（速度指令から求めた加速度のフィルタ係数）
- `shot_cycle_belt_spinup_level`（1〜4shot cycle時にbeltをONするレベル）
- `belt_shot_delay_sec`（ローラ高回転開始からFEED開始までの最短待機時間[s]）
- `dribble_position_rad``open_position_rad``custom_position_rad``feed_position_rad`
- feed_duration_sec
- `opening_max_velocity_rad_s`
- `feeding_max_velocity_rad_s`
- `returning_max_velocity_rad_s`
- `dribbling_max_velocity_rad_s`
- `opening_max_acceleration_rad_s2`
- `dribbling_max_acceleration_rad_s2`

更新値はまとめて検証される。位置は有限値保持時間と通常RPMは0以上（`slow_fire_dribble_rpm`のみ符号付き）区間速度は正の
有限値でなければ更新全体を拒否する。動作中に位置または速度を変更した場合は現在の
指令角度を始点として軌道を再計算する。

```bash
ros2 param set /dribble_controller_node dribble_on_rpm 1000
ros2 param set /dribble_controller_node shot_cycle_belt_spinup_level 2
ros2 param set /dribble_controller_node belt_shot_delay_sec 0.0
ros2 param set /dribble_controller_node dribble_position_rad 0.4
ros2 param set /dribble_controller_node feeding_max_velocity_rad_s 3.5
```

YAML全体も実行中に再読み込みできる。

```bash
ros2 param load /dribble_controller_node \
  ros2_ws/src/robot_bringup/config/dribble_controller.yaml
```

topiclogical IDQoS指令周期はnode再起動が必要なparameterである。同じ値なら
YAML全体の読み込みを許可し変更されている場合だけ拒否する。
