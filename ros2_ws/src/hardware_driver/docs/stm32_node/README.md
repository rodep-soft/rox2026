# stm32_node

STM32とのheartbeat、LED指令、limit switch受信を担当する。belt・dribbleの
モーターRPM制御には使用しない。

## 関連ファイル

- node: `hardware_driver/src/nodes/stm32_node.cpp`
- protocol宣言: `hardware_driver/include/stm32_driver/stm32_protocol.hpp`
- protocol実装: `hardware_driver/src/protocol/stm32_protocol.cpp`
- 設定: `robot_bringup/config/stm32_driver.yaml`

## 通信

| 方向 | CAN ID | DLC | 内容 |
|---|---:|---:|---|
| RDK→STM32 | `0x101` | 0 | heartbeat |
| STM32→RDK | `0x100` | 0 | heartbeat応答 |
| RDK→STM32 | `0x201` | 1 | `/led/cmd`のUInt8 |
| STM32→RDK | `0x202` | 1 | limit switch 8 bit |

standard data frameだけを受け、上記受信ID以外は早期returnする。

## limit switch

受信byteのbit 0〜7を`UInt8MultiArray.data[0]`〜`[7]`へ展開する。各要素は0か1。
Spring controllerはYAMLの`limit_switch_index`で使用するbitを選ぶ。

## heartbeat

`keep_alive_period_ms`周期で`0x101`を送る。最終`0x100`応答から`timeout_ms`を超えると
WARNを一度出す。応答復帰で内部timeout状態は解除されるが、現在は復帰INFO、
診断topic、他nodeを停止する連携はない。

## 旧motor protocol

protocolヘッダには`0x230`〜`0x232`など旧RPM用定数と関数が残るが、現在のnodeは
RPM topicを作成せず、そのencode/decode関数も呼ばない。新しい処理を読む際に
「定数がある＝使用中」と判断しないこと。

## 調査

limit switchが出ない場合は`candump can0`でID `0x202`とDLC 1を確認し、その後
`ros2 topic echo /limit_switches`でbit展開を確認する。
