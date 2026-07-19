# belt_controller

## 概要

`belt_controller_node` は `/joy_second` のボタン入力から belt 用モータの RPM 指令を作り、`/belt/rpm` に publish するノード。

## 入出力

### Subscribe

- `/joy_second` (`sensor_msgs/msg/Joy`)

### Publish

- `/belt/rpm` (`std_msgs/msg/Int16`)

## 現在の動作

起動直後の状態は `Stop`。

`/joy_second` を受信したとき、`enable_button` とモードボタンが同時に押されている場合だけ、現在のモードと RPM 値を更新する。

- `enable_button + stop_button`: `Stop`
- `enable_button + circle_button`: `Angle1High`
- `enable_button + cross_button`: `Angle1Low`
- `enable_button + triangle_button`: `Angle2JHigh`
- `enable_button + square_button`: `Angle2Low`

`enable_button` が押されていない場合、または `enable_button` だけが押されている場合は、保持している前回のモードと RPM 値を変更しない。

`/joy_second` を受信するたびに、保持している現在の RPM 値を publish する。新しい操作入力がない `/joy_second` を受信した場合も、前回の RPM 値を publish する。

`/joy_second` 自体を受信していない間は callback が呼ばれないため、このノードから新しい RPM は publish されない。

## RPM 値

RPM 値はパラメータから取得する。

- `stop_rpm`
- `circle_rpm`
- `cross_rpm`
- `triangle_rpm`
- `square_rpm`

## パラメータ

`config/config.yaml` で設定する。

`robot_bringup/launch/roller_belt.launch.py` と組み合わせる場合は、
`roller_belt_can_packer_node` の `belt_rpm_topic` を `/belt/rpm` に設定して
STM向けCANフレームへパックする。

- `enable_button`
- `stop_button`
- `circle_button`
- `cross_button`
- `triangle_button`
- `square_button`
- `stop_rpm`
- `circle_rpm`
- `cross_rpm`
- `triangle_rpm`
- `square_rpm`
