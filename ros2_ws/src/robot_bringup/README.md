# robot_bringup

ROX2026のlaunchファイルとパラメーターYAMLを管理するROS 2パッケージです。controllerやdriverの実装は持たず、用途ごとにどのノードをどの設定で起動するかを定義します。

## ディレクトリ

```text
robot_bringup/
├── config/             # controller、driver、センサーのパラメーター
├── launch/
│   ├── controllers/    # controller単位のlaunch
│   ├── hardware/       # SocketCAN、VESC、EduLite 05、STM32
│   ├── input/          # joy_node、joy_controller
│   └── test/           # 機構別の実機試験
└── scripts/            # 診断、カメラ変換、解析補助
```

## 主要launch

| launch | 用途 |
|---|---|
| `manual_robot.launch.py` | 手動操作、全controller、hardware、Foxglove |
| `game1.launch.py` | Game 1制御 |
| `game2_auto.launch.py` | Game 2自動照準。手動系・カメラ系を個別に無効化可能 |
| `pk_auto.launch.py` | PK用の手動選択・自動照準 |
| `game3_robot.launch.py` | Game 3用Joy設定と機構構成 |
| `hardware/hardware.launch.py` | SocketCANと全hardware driver |

### 手動操作

```bash
ros2 launch robot_bringup manual_robot.launch.py
```

主な引数:

| 引数 | 既定値 | 説明 |
|---|---|---|
| `can_interface` | `can0` | SocketCANインターフェース |
| `enable_foxglove` | `true` | Foxglove Bridgeを起動 |
| `foxglove_port` | `8765` | Foxglove WebSocketポート |

### Game 2自動照準

```bash
ros2 launch robot_bringup game2_auto.launch.py
```

既定では手動操作系、ビジョン、自動照準を起動します。機構を動かさず照準だけ確認する場合は次のように実行します。

```bash
ros2 launch robot_bringup game2_auto.launch.py test_alignment_only:=true
```

主な引数は `enable_manual`、`enable_vision`、`enable_game2_auto`、`test_alignment_only`、`can_interface` です。カメラの既定値はMIPI channel 1、10 fps、180度回転、ROI 800×480、AprilTagサイズ0.18 mです。

### PK自動照準

```bash
ros2 launch robot_bringup pk_auto.launch.py
```

構成とカメラ引数はGame 2と同様で、`pk_auto_aim.yaml` を使用します。

### Game 3

```bash
ros2 launch robot_bringup game3_robot.launch.py
```

`game3_joy_controller.yaml` ではL2+R2をShot Cycleへ割り当て、通常・低速Spring発射要求を無効化します。

## Hardware構成

```bash
ros2 launch robot_bringup hardware/hardware.launch.py can_interface:=can0
```

このlaunchは、送信socketを1つ、VESC・STM32・EduLite用のフィルター付き受信socketをそれぞれ1つ起動します。その後、3種類のdriverを各YAMLで起動します。

| driver | 設定 | 主な担当 |
|---|---|---|
| `vesc_driver` | `vesc_driver.yaml` | 上ベルト、下ベルト、ドリブルローラー |
| `edulite05_driver` | `edulite05_driver.yaml` | 4輪、Spring、ドリブル姿勢 |
| `stm32_driver` | `stm32_driver.yaml` | heartbeat、LED、リミットスイッチ、IMU |

CANインターフェースはROS 2起動前に作成され、UPしている必要があります。

```bash
ip -details -statistics link show can0
```

## 機構別テスト

| launch | 対象 |
|---|---|
| `test/robot_belt_dribble.launch.py` | VESC、ベルト、ドリブルローラー |
| `test/robot_mecanum.launch.py` | 4輪EduLite、Heading Hold、メカナム |
| `test/robot_dribble_position.launch.py` | STM32、Spring、ドリブル姿勢 |
| `test/belt_hardware.launch.py` | ベルトhardware単体 |
| `test/hardware_test.launch.py` | hardware driverの組み合わせ確認 |

通常launchと機構別test launchを同じCANインターフェースで同時起動しないでください。送信ノードやdriverが重複します。

## 設定ファイル

| YAML | 対象 |
|---|---|
| `joy_controller.yaml` | ボタン・軸、入力timeout、速度・加減速制限 |
| `mecanum_controller.yaml` | 機体寸法、車輪ID、速度上限 |
| `heading_hold.yaml` | IMU姿勢保持と速度feed-forward |
| `belt_controller.yaml` | 上下ベルトのレベル別RPM |
| `dribble_controller.yaml` | ローラー、姿勢、Shot Cycle、ボール検出 |
| `spring_controller.yaml` | 原点復帰、通常・低速発射 |
| `led_controller.yaml` | LED更新周期と表示保持時間 |
| `game2_auto_aim.yaml` / `pk_auto_aim.yaml` | AprilTag配置、照準、timeout |
| `vesc_driver.yaml` | VESC ID、RPM・電流制御、feedback timeout |
| `edulite05_driver.yaml` | EduLite ID、制御モード、位置基準 |
| `stm32_driver.yaml` | CAN topic、heartbeat、IMU有効期限 |

YAML最上位のノード名は、launchで指定するノード名と一致させてください。

## 起動後の確認

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /system/emergency_stop
ros2 topic echo /hardware/limit_switches
ros2 topic hz /imu/data
ros2 topic hz /vesc/state_array
ros2 topic hz /edulite/state_array
```

## 設定変更時の注意

1. RPM、rad/s、rad、m、ms、sの単位を確認します。
2. logical IDと実機のCAN IDを混同しないでください。
3. YAMLを変更したら対象パッケージを再ビルドし、使用中のinstallをsourceし直します。
4. 機構別launchで1系統ずつ確認してから統合launchを使用します。
5. ビルド成功、CANログ、ROSトピック、実機動作を別々の検証結果として記録します。

個々の制御ロジックは[robot_controller](../robot_controller/README.md)、CAN変換は[hardware_driver](../hardware_driver/README.md)、操作方法は[joy_controller](../joy_controller/README.md)を参照してください。
