# dribble_position_controller

dribble位置用EduLite 05へ目標radを繰り返し送り、feedback到達を確認しながら
手動位置移動とshot cycleを進めるnodeである。

## 関連ファイル

- 実装: `src/dribble_position_controller.cpp`
- 宣言: `include/dribble_position_controller/dribble_position_controller.hpp`
- 設定: `robot_bringup/config/dribble_position_controller.yaml`
- 起動: `robot_bringup/launch/controllers/dribble_position_controller.launch.py`

callbackは目標と状態を設定し、実際の再送・timeout監視は`watchdog_callback()`が行う。
`set_target_position()`が各移動の共通入口である。

## 入出力

| 種別 | topic | 型 | 内容 |
|---|---|---|---|
| sub | `/dribble/position_mode` | `UInt8` | 手動位置選択 |
| sub | `/operation_mode` | `UInt8` | mode別の自動移動 |
| sub | `/emergency_stop` | `Bool` | shot中断・DRIBBLE復帰 |
| sub | `/shot_cycle/start` | `Bool` | RPM準備後の開始要求 |
| sub | `/dribble/position_feedback` | `Float32` | EduLite現在位置[rad] |
| pub | `/dribble/position_command` | `Float32` | EduLite目標位置[rad] |
| pub | `/shot_cycle/running` | `Bool` | Joyへ実行中状態 |
| pub | `/shot_cycle/complete` | `Bool` | 正常復帰完了パルス |

## shot cycle

```text
start
 → INTAKEへ移動
 → feedback到達
 → SHOOTへ移動
 → feedback到達
 → shoot_to_dribble_delay_sec保持
 → DRIBBLEへ復帰
 → feedback到達
 → running=false、complete=true
```

開始には設定正常、非常停止なし、SHOT_CYCLE、待機中、未実行のすべてが必要。
条件不成立時は最初の拒否理由をログへ出す。

## watchdogと失敗時動作

移動中は`command_period_ms`周期で同じ目標を再送する。feedbackが
`feedback_timeout_sec`途絶えるか、移動が`move_timeout_sec`を超えるとshotを中断し、
DRIBBLEへ一度だけ復帰を試みる。

DRIBBLE復帰も失敗した場合はIDLEへ戻して指令再送を止める。
`/shot_cycle/complete`は出さないため、JoyはSHOT_CYCLE待機に残る。

移動時間がtimeoutの半分を超え、誤差が許容外なら「収束していない」WARNを1秒周期で
出す。NaN・Inf feedbackとIDLE中feedbackは無視する。

## mode別動作

- STOP・emergency stop: shotを中断しDRIBBLEへ復帰。
- BELT_ONLY: shotを中断しOPENへ移動。
- BELT_ONLYからDRIVE: DRIBBLEへ復帰。
- DRIVE・SHOT_CYCLE待機中: DRIBBLE、INTAKE、SHOOT、OPENの手動移動を許可。

## parameter

4位置は有限かつ`[-pi, pi]`、toleranceは0より大きい必要がある。tolerance不正は
0.02 rad、command period不正は20 ms、QoS不正は1へ補正する。それ以外の位置・時間
設定が不正なら位置指令を出さない。

## 確認方法

```bash
ros2 topic echo /dribble/position_command
ros2 topic echo /dribble/position_feedback
ros2 topic echo /shot_cycle/running
```

位置方向を変えるときはYAMLのrad値とEduLiteの実機原点・回転方向を同時に確認する。
