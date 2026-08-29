# stm32_node

STM32とのheartbeat、LED指令、limit switch受信を担当する。belt・dribbleの
モーターRPM制御には使用しない。

## 関連ファイル

- node: `hardware_driver/src/nodes/stm32_node.cpp`
- protocol宣言: `hardware_driver/include/stm32_driver/stm32_protocol.hpp`
- protocol実装: `hardware_driver/src/protocol/stm32_protocol.cpp`
- 設定: `robot_bringup/config/stm32_driver.yaml`

## 通信

CANの送受信トピックは`can_tx_topic`と`can_rx_topic`で設定する。既定値はそれぞれ
`/socketcan_bridge/tx`と`/socketcan_bridge/rx`である。

| 方向 | CAN ID | DLC | 内容 |
|---|---:|---:|---|
| RDK→STM32 | `0x101` | 0 | heartbeat |
| STM32→RDK | `0x100` | 0 | heartbeat応答 |
| RDK→STM32 | 0x201 | 5 | /hardware/led_cmdのUInt64 |
| STM32→RDK | `0x310` | 1 | limit switch 8 bit |
| STM32→RDK | `0x320` | 8 | quaternion X, Y, Z, W（int16 LE、1/16384） |
| STM32→RDK | `0x321` | 6 | angular velocity X, Y, Z（int16 LE、1/16 deg/s） |
| STM32→RDK | `0x322` | 6 | linear acceleration X, Y, Z（int16 LE、1/100 m/s²） |

LED command uses DLC 5: data[0] is the display mode, data[1] is the status flags,
and data[2] through data[4] contain nine 2-bit grid states. The ROS topic type is
std_msgs/msg/UInt64 (little endian).

standard data frameだけを受け、上記受信ID以外は早期returnする。

IMUの各CANフレームは最新値として保持し、100 Hzのquaternion受信を契機に、
最新の角速度・並進加速度と合わせて`sensor_msgs/msg/Imu`を`/imu/data`へpublishする。
角速度または並進加速度が未受信、もしくは`imu_component_timeout_ms`を超えて
更新されていない場合は、対応する`covariance[0]`を`-1`として無効であることを示す。

## limit switch

CANデータ先頭byteを`UInt8`として`/limit_switchs`へpublishする。Spring controllerが
`limit_switch_bit_offset`で指定したbitを取り出し、0ならOFF、1ならONとして扱う。

## heartbeat

`keep_alive_period_ms`周期で`0x101`を送る。最終`0x100`応答から`timeout_ms`を超えると
WARNを一度出す。応答復帰で内部timeout状態は解除されるが、現在は復帰INFO、
診断topic、他nodeを停止する連携はない。

## 旧motor protocol

protocolヘッダには`0x230`〜`0x232`など旧RPM用定数と関数が残るが、現在のnodeは
RPM topicを作成せず、そのencode/decode関数も呼ばない。新しい処理を読む際に
「定数がある＝使用中」と判断しないこと。

## 調査

limit switchが出ない場合は`candump can0`でID `0x310`と対象byteを確認し、その後
`ros2 topic echo /limit_switchs`で値を確認する。
