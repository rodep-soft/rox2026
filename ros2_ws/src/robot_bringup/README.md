# robot_bringup

操作入力・ハードウェア層と、機構制御層を別々に起動するためのlaunchを提供します。

## 事前準備

リポジトリのルートディレクトリでDockerコンテナを起動し、ワークスペースをビルドします。

```bash
docker compose up -d
docker compose exec ros2_rox2026 bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws
colcon build --packages-up-to robot_bringup
source install/setup.bash
```

`can0` を有効化し、ジョイスティックを `/dev/input/js0` として接続してください。

## 起動方法

以下の2つを別ターミナルで起動します。各ターミナルで `/opt/ros/humble/setup.bash` と `install/setup.bash` を source してください。

### ターミナル1: 操作入力とハードウェア層

```bash
ros2 launch robot_bringup hardware_joy.launch.py
```

起動するノードは次のとおりです。

- `joy_node`
- `joy_controller_node`
- `socketcan_bridge`
- `stm32_node`
- 4輪、spring、dribble位置用のEduLite driver

### ターミナル2: 機構制御層

```bash
ros2 launch robot_bringup controllers.launch.py
```

起動するノードは次のとおりです。

- `mecanum_controller_node`
- `spring_controller_node`
- `dribble_controller_node`
- `belt_controller_node`
- `dribble_position_controller_node`

### ドリブル位置だけを確認する場合

ターミナル1で `hardware_joy.launch.py` を起動したうえで、ターミナル2では次を実行します。

```bash
ros2 launch robot_bringup dribble_position_controller.launch.py
```

位置feedbackは `/dribble/position_feedback` で受信します。

## 注意

`hardware_joy.launch.py` はEduLite driverを初期化してモータをenableします。実機で起動する前に、車輪を浮かせるなど安全を確保してください。
