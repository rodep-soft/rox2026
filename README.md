# rox2026
ROX2026用レポジトリ

## Branch Rules

- `main`への直接pushは禁止(PR only) <== Ruleset入れてあるので大会前に削除
- Merge前にビルドチェックを行う
- branchを必ず切って作業する

---
## RDKX5
### ROS2パッケージ一覧

| Package | 説明 |
|---------|------|
| joy_controller | Joy入力を機構指令とoperation modeへ変換 |
| robot_controller | 各機構の状態遷移と制御判断 |
| hardware_driver | 機構指令とVESC・EduLite 05・STM32のCAN frameを相互変換 |
| robot_bringup | 通常起動・機構別test用のlaunchとparameter |

---

### 制御経路

```text
joy_controller → robot_controller → hardware_driver → CAN機器
```

- mecanum・spring・dribble positionはEduLite 05で制御する。
- underbelt・upperbelt・dribble回転はVESCで制御する。
- STM32はlimit switch、LED、heartbeatを担当する。
- CAN送受信は`ros2_socketcan`の`/socketcan_bridge/tx`と
  `/socketcan_bridge/rx`を使用する。

topic名・型の詳細は以下を正とする。

- [`joy_controller/README.md`](ros2_ws/src/joy_controller/README.md)
- [`robot_controller/README.md`](ros2_ws/src/robot_controller/README.md)
- `robot_bringup/config`内のYAMLコメント

### コントローラボタン配置

ボタン・軸の番号と操作方法は
[`joy_controller/README.md`](ros2_ws/src/joy_controller/README.md)を正とする。

## stm32f303k8

### 担当する入出力

- limit switch状態の送信
- LED指令の受信
- RDK X5とのheartbeat

ベルト・dribble回転数の指令とfeedbackはSTM32を経由せず、VESC driverが
直接担当する。
