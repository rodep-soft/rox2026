# vesc_node

3台のVESCを1ノードで管理し、上位ノードのlogical ID付きRPM指令をVESC CANフレームへ変換する。

## ROSインターフェース

| 種別 | 既定名 | 型 | 用途 |
|---|---|---|---|
| subscribe | `/vesc/target` | `actuator_msgs/msg/ActuatorTarget` | 1台の機械RPM指令 |
| subscribe | `/vesc/target_array` | `actuator_msgs/msg/ActuatorTargetArray` | 複数台の機械RPM指令 |
| publish | `/vesc/state` | `actuator_msgs/msg/ActuatorState` | CAN受信時に更新された1台の状態 |
| publish | `/vesc/state_array` | `actuator_msgs/msg/ActuatorStateArray` | 全VESCの周期状態 |

VESCでは`target`と`state.velocity`の単位を機械RPMとして扱う。上位ノードはlogical IDだけを使用し、VESC CAN controller IDとERPM変換はhardware_driver内に閉じ込める。

## モーター別パラメーター

| パラメーター | 内容 |
|---|---|
| `logical_id` | ROS側で使用する一意なID |
| `controller_id` | VESC CAN ID |
| `command_timeout_ms` | 指令断で0 RPMへ移る時間 |
| `feedback_timeout_ms` | 接続切れと判定する時間 |
| `max_rpm` | 機械RPM指令の絶対値上限 |
| `rpm_slew_rate` | 1秒あたりの機械RPM変化量 |

現在の割り当てはupper beltがlogical ID 10、under beltが11、dribbleが12である。