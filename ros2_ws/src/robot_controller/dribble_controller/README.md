# dribble_controller

ドリブル機構のローラー回転と姿勢角度を1ノードで制御する。

## 入出力

| 種別 | topic | 内容 |
|---|---|---|
| sub | `/dribble/enabled` | ローラー回転ON/OFF |
| sub | `/dribble/position_mode` | `DRIBBLE`、`OPEN`、`FEED`姿勢 |
| sub | `/shot_cycle/request` | `OPEN → FEED → DRIBBLE`自動動作 |
| sub | `/belt/mode` | beltの現在モード把握（0=STOP、1〜4=LEVEL） |
| sub | `/emergency_stop` | ローラー停止、DRIBBLE姿勢復帰 |
| pub | `/vesc/target` | ローラー目標RPM |
| pub | `/edulite/target` | 姿勢目標角度[rad] |
| pub | `/belt/mode` | shot cycle時のbelt自動ON/OFF |
| pub | `/shot_cycle/state` | サイクル進行フェーズ（LED通知用） |

## shot cycleの動作

`/shot_cycle/request`にtrueが届くと、次の手順でボールを射出する。

### beltがSTOPの場合（自動spin-up）

```
[ボタン押下]
    │
    ▼
belt を LEVEL_{shot_cycle_belt_spinup_level} に自動ON
    │
    ▼  belt_spinup_delay_sec 秒待機（スピンアップ）
    │
    ▼
DRIBBLE → OPEN（アーム開く）
    │  open_duration_sec 保持
    ▼
OPEN → FEED（押し込み射出）
    │  feed_duration_sec 保持
    ▼
FEED → DRIBBLE（アーム復帰）
    │
    ▼
belt を自動STOP
```

### beltが既に回っている場合（即時shot）

```
[ボタン押下]
    │
    ▼
DRIBBLE → OPEN → FEED → DRIBBLE（belt はそのまま）
```

手動でアーム位置を変更すると shot cycle は中断される。
emergency stopが有効な場合はshot cycle要求を無視する。

## 実行中に変更できるparameter

- `dribble_on_rpm`
- `shot_cycle_belt_spinup_level`（1〜4、shot cycle時にbeltをONするレベル）
- `belt_spinup_delay_sec`（beltがSTOPからONになった後の待機時間[s]）
- `dribble_position_rad`、`open_position_rad`、`feed_position_rad`
- `open_duration_sec`、`feed_duration_sec`
- `opening_max_velocity_rad_s`
- `feeding_max_velocity_rad_s`
- `returning_max_velocity_rad_s`

更新値はまとめて検証される。位置は有限値、保持時間とRPMは0以上、区間速度は正の
有限値でなければ更新全体を拒否する。動作中に位置または速度を変更した場合は、現在の
指令角度を始点として軌道を再計算する。

```bash
ros2 param set /dribble_controller_node dribble_on_rpm 1000
ros2 param set /dribble_controller_node shot_cycle_belt_spinup_level 2
ros2 param set /dribble_controller_node belt_spinup_delay_sec 0.8
ros2 param set /dribble_controller_node dribble_position_rad 0.4
ros2 param set /dribble_controller_node feeding_max_velocity_rad_s 3.5
```

YAML全体も実行中に再読み込みできる。

```bash
ros2 param load /dribble_controller_node \
  ros2_ws/src/robot_bringup/config/dribble_controller.yaml
```

topic、logical ID、QoS、指令周期はnode再起動が必要なparameterである。同じ値なら
YAML全体の読み込みを許可し、変更されている場合だけ拒否する。
