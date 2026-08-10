# mecanum_controller_node

機体速度`geometry_msgs/msg/Twist`を4輪のEduLite角速度へ変換するnodeである。
メカナム逆運動学、非常停止、最大50 rad/sへの比率維持制限を担当する。

## 関連ファイル

- 実装: `src/mecanum_controller_node.cpp`
- 宣言: `include/mecanum_controller/mecanum_controller_node.hpp`
- 設定: `robot_bringup/config/mecanum_controller.yaml`
- 起動: `robot_bringup/launch/controllers/mecanum_controller.launch.py`

`publish_wheel_commands()`へ計算とpublishを集約している。cmd_velを受信すると即時計算して
publishする。非常停止の開始・解除も受信時に即時反映し、非常停止中だけ全輪ゼロ指令を
`emergency_stop_period_ms`周期で再送する。通常走行中はtimerによる再送を行わない。

## 入出力

| 種別 | topic | 型 |
|---|---|---|
| sub | `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` |
| sub | `/emergency_stop` | `std_msgs/msg/Bool` |
| pub | `/edulite/target_array` | `actuator_msgs/msg/ActuatorTargetArray` |

## 計算順

1. 非常停止中なら機体速度の全成分を0にする。
2. 車輪半径と機体寸法を使い、前左・前右・後左・後右の角速度を計算する。
3. 4輪中で絶対値が最大の角速度を求める。
4. 最大値が上限を超えた場合、`上限 / 最大値`を全輪へ掛ける。
5. 4輪の目標角速度[rad/s]をpublishする。

車輪直径150 mmは半径`0.075 m`として計算する。全輪を同じ比率で縮小するため、
斜め移動や旋回を含む指令でも移動方向と車輪間比率を保つ。

## 入力異常とparameter補正

cmd_velの`linear.x`、`linear.y`、`angular.z`のどれかがNaN・Infなら、最後の指令を
ゼロへ置き換えて全輪0をpublishする。

半径、寸法、上限、QoSには起動時検証がある。不正値はWARNまたはERROR後に安全な
既定値へ補正し、nodeは起動を続ける。上限はEduLite仕様に合わせて50 rad/s以下。

## 確認方法

```bash
ros2 topic echo /edulite/target_array
ros2 topic pub --once /mecanum/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

車輪を浮かせた状態で前左・前右・後左・後右のtopicと実機対応、回転方向を確認する。
