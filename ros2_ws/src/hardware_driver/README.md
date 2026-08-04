# hardware_driver

`robot_controller`が出す機構指令をCANフレームへ変換し、CANから受けた状態を
ROS 2のfeedbackへ戻すパッケージである。CAN ID、フレーム形式、エンディアン、
モーター固有のERPM変換はこのパッケージ内で扱う。

## node別の詳細資料

- [vesc_node](docs/vesc_node/README.md)
- [stm32_node](docs/stm32_node/README.md)
- [edulite05_node](docs/edulite05_node/README.md)

このREADMEはhardware全体の割り当てを示す。CAN処理を追ってsourceを読む場合は、
各nodeの詳細READMEを先に参照する。

## 接続構成

| node | 実機 | 台数 | 主な役割 |
|---|---|---:|---|
| `vesc_node` | 赤ブラシ用VESC | 3 | underbelt、upperbelt、dribbleのRPM制御 |
| `edulite05_node` | EduLite 05 | 6 | mecanum 4輪、Spring速度、dribble位置制御 |
| `stm32_node` | STM32 | 1 | リミットスイッチ、LED、heartbeat |

全nodeは`/socketcan_bridge/tx`へ`can_msgs/msg/Frame`をpublishし、
`/socketcan_bridge/rx`をsubscribeする。SocketCANとの接続と起動条件は
`robot_bringup`が担当する。

## 実機割り当て

| 機構 | driver node | ID | mode | command | feedback |
|---|---|---:|---|---|---|
| upperbelt | `vesc_upper_belt_driver` | 51 | RPM | `/upperbelt/target/rpm` | `/upperbelt/current/rpm` |
| underbelt | `vesc_under_belt_driver` | 52 | RPM | `/underbelt/target/rpm` | `/underbelt/current/rpm` |
| dribble回転 | `vesc_dribble_driver` | 50 | RPM | `/dribble/target/rpm` | `/dribble/current/rpm` |
| mecanum FL | `edulite05_fl_driver` | `0x01` | Velocity | `/mecanum/fl/vel_command` | feedback非publish |
| mecanum FR | `edulite05_fr_driver` | `0x02` | Velocity | `/mecanum/fr/vel_command` | feedback非publish |
| mecanum RL | `edulite05_rl_driver` | `0x03` | Velocity | `/mecanum/rl/vel_command` | feedback非publish |
| mecanum RR | `edulite05_rr_driver` | `0x04` | Velocity | `/mecanum/rr/vel_command` | feedback非publish |
| Spring | `edulite05_spring_driver` | `0x0A` | Velocity | `/spring/vel_command` | feedback非publish |
| dribble位置 | `edulite05_dribble_position_driver` | `0x38` | Position | `/dribble/position_command` | `/dribble/position_feedback` |

VESC IDは`robot_bringup/config/vesc_driver.yaml`、EduLite IDは
`robot_bringup/config/edulite05_driver.yaml`を正とする。実機交換時は、最初に
この表とYAMLが一致していることを確認する。

## vesc_node

### 入出力

RPM指令とfeedbackはどちらも機械RPMで、型は`std_msgs/msg/Int16`である。
VESC CAN上のERPMへ送るときは、コード内のモーター極数14から求めた極対数7を
掛ける。STATUS_1で受けたERPMは7で割り、四捨五入後にInt16範囲へ制限する。

送信するSET_RPMフレームはextended CAN ID
`(3 << 8) | controller_id`、DLC 4、big-endianの符号付きERPMである。
受信はpacket ID 9のSTATUS_1だけを対象とし、下位8 bitのcontroller IDが
自nodeと一致するフレームだけを処理する。

### 指令処理

1. target RPMを受ける。
2. `abs(target) > max_rpm`ならWARNを出して破棄する。
3. 有効な指令を保持し、20 ms周期で`rpm_slew_rate`の範囲内でVESCへ再送する。
4. 最後の有効指令から`command_timeout_ms`を超えた場合は0 RPMを送る。
5. 新しい指令が来れば通常のRPM送信へ復帰する。

一度も指令を受けていない間はSET_RPMを送らない。controller側は周期publishする
設計なので、通常起動では停止時も0 RPM指令が届く。

### feedback処理

起動後または最後のSTATUS_1から`feedback_timeout_ms`を超えるとWARNを1回出し、
current RPMのpublishを停止する。0や古い値はpublishしない。feedbackが復帰すると
INFOを出し、current RPMの周期publishを再開する。

### parameter

| parameter | 型 | 内容・制約 |
|---|---|---|
| `can_pub_topic` | string | CAN送信topic |
| `can_sub_topic` | string | CAN受信topic |
| `target_rpm_topic` | string | 機械RPM指令topic |
| `current_rpm_topic` | string | 機械RPM feedback topic |
| `controller_id` | int | VESC CAN ID。0〜255 |
| `command_timeout_ms` | int | 指令断で0 RPMへ移る時間。1以上 |
| `feedback_timeout_ms` | int | feedback publishを止めるまでの時間。1以上 |
| `max_rpm` | int | 受理するRPM絶対値の上限。1〜32767 |
| `rpm_slew_rate` | double | 1秒あたりのRPM変化量 |

範囲外parameterはnode起動時に例外となり、nodeは起動しない。

## edulite05_node

### 起動と初期化

`runmode`に応じてVelocityまたはPosition用のCAN frame生成器を作る。起動時に
100 ms待機してから、run mode、制限値、enableなどの初期化frameを50 ms間隔で
送る。無効な`runmode`ではmotorインスタンスを作れないため、YAMLでは
`Velocity`か`Position`だけを使用する。

feedback CAN IDのmotor IDが自nodeと一致する場合だけ処理する。電源投入通知を
受けた場合と、enable済みなのにfeedbackがdisableを示した場合は初期化frameを
再送する。後者ではcallback内で100 ms待機する。

正常終了時はdisable frameを1回送る。プロセス強制終了や電源断では、この処理が
実行されない可能性がある。

### mode別処理

- `Velocity`: `std_msgs/msg/Float32`のrad/sを速度制御frameへ変換する。
- `Position`: `std_msgs/msg/Float32`のradを位置制御frameへ変換する。
- `is_requested_fb_pub=true`: Velocityならrad/s、Positionならradをfeedbackへ出す。

現在の設定ではdribble位置だけfeedbackをpublishし、mecanumとSpringはpublishしない。

### parameter

| parameter | 型 | 内容 |
|---|---|---|
| `sub_cmd_topic_name` | string | controllerから受ける指令topic |
| `pub_can_topic_name` | string | CAN送信topic |
| `sub_can_topic_name` | string | CAN受信topic |
| `pub_fb_topic_name` | string | feedback topic |
| `motor_id` | uint8 | EduLite 05のmotor ID |
| `runmode` | string | `Velocity`または`Position` |
| `is_requested_fb_pub` | bool | feedbackをROS 2へpublishするか |

電流、加速度、最大速度などのEduLite初期値は現在
`edulite05_protocol.hpp`内の定数であり、YAML parameterではない。

## stm32_node

STM32はbelt・dribbleのRPM制御には使用しない。

### 有効な通信

| 方向 | CAN ID | 内容 |
|---|---:|---|
| RDK → STM32 | `0x101` | heartbeat。dataなし |
| STM32 → RDK | `0x100` | heartbeat応答。dataなし |
| RDK → STM32 | `0x201` | LED指令。1 byte |
| STM32 → RDK | `0x310` | limit switch。指定byteを使用 |

STM32 nodeはCANデータ先頭byteを`std_msgs/msg/UInt8`として`/limit_switchs`へpublishする。
spring controllerは`limit_switch_bit_offset`で指定したbitを取り出し、0ならOFF、1ならONとして扱う。

`keep_alive_period_ms`周期でheartbeatを送り、最後の応答から`timeout_ms`を超えると
WARNを1回出す。現在はtimeoutを通知するROS topicや、他機構を直接停止する処理はない。

ヘッダには旧STM32モーターRPM用CAN IDとframe関数が残っているが、現在の
`stm32_node`はRPM topicを作らず、RPM frameも送受信しない。

## トラブルシュート

| 症状 | 最初に確認するもの |
|---|---|
| VESCが回らない | target RPM、`max_rpm`超過WARN、YAMLのcontroller ID |
| current RPMが出ない | VESC STATUS_1、feedback timeout WARN、極数設定 |
| EduLiteが動かない | motor ID、runmode、起動時初期化frame、CAN extended ID |
| dribble位置feedbackがない | `is_requested_fb_pub=true`、motor ID `0x38` |
| limit switchが出ない | STM32 CAN ID `0x310`、CANデータ先頭byte、`/limit_switchs` |
| STM32 timeout WARN | `0x100` heartbeat応答と`timeout_ms` |

実機を動かす前に、同一CAN bus上でVESC IDとEduLite IDが意図した実機へ割り当てられて
いることを確認する。ID変更後はcontroller側ではなく、対応するbringup YAMLを更新する。
