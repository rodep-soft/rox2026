# belt_controller

## 概要

`belt_controller_node` は `/joy` のボタン入力から belt 用モータの PWM 指令を作り、belt側入力の `/belt/rpm_value` に publish するノード。

## 入出力

### Subscribe

- `/joy` (`sensor_msgs/msg/Joy`)

### Publish

- `/belt/rpm_value` (`std_msgs/msg/Int16`)

## 現在の動作

起動直後の状態は `Stop`。

`/joy` を受信したとき、`enable_button` とモードボタンが同時に押されている場合だけ、現在のモードと PWM 値を更新する。

- `enable_button + stop_button`: `Stop`
- `enable_button + circle_button`: `Angle1High`
- `enable_button + cross_button`: `Angle1Low`
- `enable_button + triangle_button`: `Angle2JHigh`
- `enable_button + square_button`: `Angle2Low`

`enable_button` が押されていない場合、または `enable_button` だけが押されている場合は、保持している前回のモードと PWM 値を変更しない。

`/joy` を受信するたびに、保持している現在の PWM 値を publish する。新しい操作入力がない `/joy` を受信した場合も、前回の PWM 値を publish する。

`/joy` 自体を受信していない間は callback が呼ばれないため、このノードから新しい PWM は publish されない。

## PWM 値

PWM 値はパラメータから取得する。

- `stop_pwm`
- `circle_pwm`
- `cross_pwm`
- `triangle_pwm`
- `square_pwm`

publish 前に `0` から `255` の範囲に丸める。

## パラメータ

`config/config.yaml` で設定する。

- `enable_button`
- `stop_button`
- `circle_button`
- `cross_button`
- `triangle_button`
- `square_button`
- `stop_pwm`
- `circle_pwm`
- `cross_pwm`
- `triangle_pwm`
- `square_pwm`
