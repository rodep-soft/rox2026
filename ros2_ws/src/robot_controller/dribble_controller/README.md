# dribble_controller

ドリブル機構のローラー回転と姿勢角度を1ノードで制御する。

## 入出力

| 種別 | topic | 内容 |
|---|---|---|
| sub | `/dribble/enabled` | ローラー回転ON/OFF |
| sub | `/dribble/position_mode` | `DRIBBLE`、`OPEN`、`FEED`姿勢 |
| sub | `/shot_cycle/request` | `OPEN → FEED → DRIBBLE`自動動作 |
| sub | `/emergency_stop` | ローラー停止、DRIBBLE姿勢復帰 |
| pub | `/vesc/target` | ローラー目標RPM |
| pub | `/edulite/target` | 姿勢目標角度[rad] |

## 実行中に変更できるparameter

- `dribble_on_rpm`
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
ros2 param set /dribble_controller_node dribble_position_rad 0.4
ros2 param set /dribble_controller_node feeding_max_velocity_rad_s 3.5
```
