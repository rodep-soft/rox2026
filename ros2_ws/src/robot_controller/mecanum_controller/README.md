# mecanum_controller_node

機体速度`geometry_msgs/msg/Twist`を4輪のEduLite角速度へ変換するnodeである。
mode制限、符号補正、車輪補正、最大50 rad/sへの比率維持制限を担当する。

## 関連ファイル

- 実装: `src/mecanum_controller_node.cpp`
- 宣言: `include/mecanum_controller/mecanum_controller_node.hpp`
- 設定: `robot_bringup/config/mecanum_controller.yaml`
- 起動: `robot_bringup/launch/controllers/mecanum_controller.launch.py`

`publish_wheel_commands()`へ処理が集約されている。cmd_vel、mode、emergency callbackは
内部値を更新した直後にこの関数を呼ぶ。

## 入出力

| 種別 | topic | 型 |
|---|---|---|
| sub | `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` |
| sub | `/operation_mode` | `std_msgs/msg/UInt8` |
| sub | `/emergency_stop` | `std_msgs/msg/Bool` |
| pub | `/edulite/target_array` | `actuator_msgs/msg/ActuatorTargetArray` |

## 計算順

1. `vx_sign`、`vy_sign`、`angular_z_sign`を掛ける。
2. STOP・emergency stopなら全成分を0にする。
3. SHOT_CYCLE・BELT_ONLYなら並進を0にし旋回だけ残す。
4. 車輪半径と機体寸法を使い、FL・FR・RL・RRの逆運動学を計算する。
5. `velocity_corrections`を車輪ごとに掛ける。
6. 最大絶対値が上限を超えたら、全輪へ同じ縮小率を掛ける。
7. 4つのrad/sをpublishする。

全輪を同じ比率で縮小するため、斜め入力でも移動方向と車輪間比率を保つ。
modeが変わった瞬間にも最後のcmd_velから再計算し、次のJoy messageを待たない。

## 入力異常とparameter補正

cmd_velの`linear.x`、`linear.y`、`angular.z`のどれかがNaN・Infなら、最後の指令を
ゼロへ置き換えて全輪0をpublishする。

半径、寸法、上限、QoS、符号、補正配列には起動時検証がある。不正値はWARN/ERROR後に
安全な既定値へ補正し、nodeは起動を続ける。上限はEduLite仕様に合わせて50 rad/s以下。

## 確認方法

```bash
ros2 topic echo /edulite/target_array
ros2 topic pub --once /mecanum/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

車輪を浮かせた状態でFL、FR、RL、RRのtopicと実機対応、符号、補正係数を確認する。
