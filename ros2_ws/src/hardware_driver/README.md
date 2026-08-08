# hardware_driver

`robot_controller`の機構指令をCANフレームへ変換し、受信CANフレームからハードウェア状態を復元するROS 2パッケージ。
CAN ID、8 byteフレーム、エンディアン、デバイス固有プロトコルはこのパッケージ内に閉じ込める。

## ノード

- `edulite05_node`: EduLite 05のvelocity、PP、CSP制御。詳細は[docs/edulite05_node/README.md](docs/edulite05_node/README.md)。
- `stm32_node`: STM32とのheartbeat、LED、limit switch通信。詳細は[docs/stm32_node/README.md](docs/stm32_node/README.md)。
- `vesc_node`: VESCのRPM指令とフィードバック。詳細は[docs/vesc_node/README.md](docs/vesc_node/README.md)。

全ノードは`/socketcan_bridge/tx`へ`can_msgs/msg/Frame`を送信し、`/socketcan_bridge/rx`から受信する。起動設定と機体ごとのIDは`robot_bringup`で管理する。
