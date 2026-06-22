# motor_can_bridge

## 概要

`motor_can_bridge_node` はモータ制御ノードから PWM topic を受け取り、CAN 送信用の `CanFrame` message に詰め替えて `/can_tx` に publish するノード。

このノードは CAN bus へ直接送信しない。

## 入出力

### Subscribe

- `/mabuchi555/pwm_value` (`std_msgs/msg/Int16`)
- `/mad_motor/pwm_value` (`std_msgs/msg/Int16`)

topic 名はパラメータで変更できる。

### Publish

- `/can_tx` (`motor_can_bridge/msg/CanFrame`)

## 現在の動作

起動直後の Mabuchi PWM と MAD motor PWM はどちらも `0`。

各 PWM topic を受信したとき、最新値と最終受信時刻を保持する。

- Mabuchi PWM は `-255` から `255` の範囲に丸める
- MAD motor PWM は `0` から `255` の範囲に丸める

timer により `send_period_ms` ごとに、Mabuchi 用と MAD motor 用の CAN frame をそれぞれ publish する。

各 PWM topic の最終受信時刻から `timeout_ms` を超えている場合、その系統の PWM は `0` として CAN frame を作る。

## CAN frame

publish する message 型は `motor_can_bridge/msg/CanFrame`。

現在の payload は 8 byte。

- `data[0]`: `0x00`
- `data[1]`: PWM の下位 byte
- `data[2]`: PWM の上位 byte
- `data[3]` から `data[7]`: `0x00`

PWM は `int16` の little-endian として格納する。

CAN ID はパラメータから取得する。

- `mabuchi_can_id`
- `mad_motor_can_id`

## パラメータ

`config/config.yaml` で設定する。

- `mabuchi_pwm_topic`
- `mad_motor_pwm_topic`
- `can_tx_topic`
- `mabuchi_can_id`
- `mad_motor_can_id`
- `send_period_ms`
- `timeout_ms`
