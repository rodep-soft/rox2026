# robstride_can

RDKからSocketCANを使ってRobstride EduLite 05を動かすための実験用packageです。

このpackageはSTM向けのPWM CAN bridgeとは別物です。`can_msgs` topicには変換せず、nodeからLinux SocketCANへ直接writeします。

実際に動いた可能性がある `rodep-soft/el05_usb_can_driver` の設計に合わせ、Robstride EL05の **Private Protocol** を使います。MIT protocolではありません。

## 前提

- Robstride側がPrivate Protocolで動くこと
- CAN bitrateはRobstride EduLite 05のdefaultである1Mbpsに合わせること
- 最初は無負荷、低速、すぐ電源を切れる状態で確認すること

Private Protocolでは29-bit extended CAN IDを使います。速度モードでは、`run_mode` をvelocity modeにしてからenableし、`spd_ref` と `limit_cur` parameterを書きます。

## CAN interface設定例

RDK側でRobstrideにつながっているinterface名を確認します。

```bash
ip link show
```

`can0` を使う場合の設定例です。

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 起動

```bash
ros2 launch robstride_can robstride_velocity.launch.py
```

configを変える場合:

```bash
ros2 launch robstride_can robstride_velocity.launch.py config_file:=/path/to/robstride_velocity.yaml
```

## 速度指令

まずvelocity modeへ切り替えてenableします。

```bash
ros2 topic pub --once /robstride_velocity_node/stop std_msgs/msg/Empty '{}'
ros2 topic pub --once /robstride_velocity_node/set_mode std_msgs/msg/UInt8 "{data: 2}"
ros2 topic pub --once /robstride_velocity_node/enable std_msgs/msg/Bool "{data: true}"
```

起動後に速度を変える場合:

```bash
ros2 topic pub /robstride_velocity_node/velocity_command std_msgs/msg/Float64MultiArray "{data: [0.5, 1.0]}" -1
```

停止させる場合:

```bash
ros2 topic pub --once /robstride_velocity_node/stop std_msgs/msg/Empty '{}'
```

node終了時にもstop commandを送る設定にしています。

## 主なparameter

- `can_interface`: 使用するSocketCAN interface。例: `can0`
- `motor_id`: Robstrideに設定されているmotor CAN ID
- `host_id`: Robstrideのstatus responseを受けるhost ID
- `current_limit_a`: velocity modeの電流制限

## 注意

RobstrideがMIT protocolに切り替わっている場合、このnodeのPrivate Protocol commandでは回りません。

実機で動かすときは、まず `candump can0` で29-bit extended frameが流れていることと、feedbackが返っていることを確認してください。
