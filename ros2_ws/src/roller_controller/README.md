# roller_controller

## 概要

`roller_controller_node` は `/joy` のボタン入力からローラー用の PWM 指令を作り、`/mabuchi555/pwm_value` に publish するノード。

## 入出力

### Subscribe

- `/joy` (`sensor_msgs/msg/Joy`)

### Publish

- `/mabuchi555/pwm_value` (`std_msgs/msg/Int16`)

## 現在の動作

起動直後の状態は `Stop`。

`/joy` を受信したとき、`enable_button` と方向ボタンが同時に押されている場合だけ、現在の回転モードと PWM 値を更新する。

- `enable_button + positive_button`: `PositiveRotate`
- `enable_button + negative_button`: `NegativeRotate`
- `enable_button + stop_button`: `Stop`

`enable_button` が押されていない場合、または `enable_button` だけが押されている場合は、保持している前回のモードと PWM 値を変更しない。

`/joy` を受信するたびに、保持している現在の PWM 値を publish する。新しい操作入力がない `/joy` を受信した場合も、前回の PWM 値を publish する。

`/joy` 自体を受信していない間は callback が呼ばれないため、このノードから新しい PWM は publish されない。

## PWM 値

PWM 値はパラメータから取得する。

- `positive_pwm`
- `negative_pwm`
- `stop_pwm`

publish 前に `-255` から `255` の範囲に丸める。

## パラメータ

`config/config.yaml` で設定する。

- `enable_button`
- `positive_button`
- `negative_button`
- `stop_button`
- `positive_pwm`
- `negative_pwm`
- `stop_pwm`
