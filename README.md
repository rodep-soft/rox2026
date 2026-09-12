# ROX2026

九州工業大学ロボットサークル RoDEP のロボット「ROX2026」向け制御ソフトウェアです。

> [!CAUTION]
> このソフトウェアは実機のモーターやアクチュエーターを動作させます。初回起動や設定変更後は、非常停止を使用できる状態にし、車輪や機構を浮かせるなど周囲の安全を確保して検証してください。

## システム構成

```text
ゲームパッド / センサー
          ↓
   joy_controller
          ↓
  robot_controller
          ↓
   hardware_driver
          ↓
 SocketCAN ── VESC / EduLite 05 / STM32
```

主なROS 2パッケージは次のとおりです。

| パッケージ | 役割 |
|---|---|
| `actuator_msgs` | アクチュエーター指令・状態・位置校正サービスの共通インターフェース |
| `joy_controller` | ゲームパッド入力を機構指令と運転モードへ変換 |
| `robot_controller` | 各機構の状態遷移、軌道生成、安全処理、自動制御 |
| `hardware_driver` | ROSメッセージとVESC・EduLite 05・STM32のCANフレームを相互変換 |
| `robot_bringup` | 実機のパラメーター、通常起動、競技モード、機構別テスト |
| `robot_msgs` | ロボット固有の制御・テレメトリメッセージ |
| `ros2_socketcan` | SocketCANとROS 2トピックのブリッジ |

モーターと周辺機器の担当は次のように分かれています。

- EduLite 05: メカナムホイール、ばね、ドリブル姿勢
- VESC: 上下ベルト、ドリブルローラー
- STM32: リミットスイッチ、LED、IMU、heartbeat

## 必要な環境

- Docker EngineおよびDocker Compose
- LinuxまたはWSL 2
- 実機接続時はSocketCAN対応CANインターフェース
- ゲームパッドやカメラなど、使用する機能に対応したデバイス

コンテナはROS 2 Humble環境を構築します。GUIツールを使用する場合、現在のCompose設定はWSLgを前提としています。

## セットアップ

```bash
git clone <repository-url> rox2026
cd rox2026
docker compose build
docker compose up -d
docker compose exec ros2_rox2026 bash
```

コンテナ内の作業ディレクトリは `/root/ros2_ws` です。

## ビルド

コンテナ内で実行します。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

ROS 2環境を導入済みのホストでは、リポジトリ直下のMakefileも利用できます。

```bash
make build
make build-pkg pkg=hardware_driver
```

## 起動

ビルド後、コンテナ内で用途に合うlaunchファイルを実行します。

```bash
# 手動操作
ros2 launch robot_bringup manual_robot.launch.py

# Game 1
ros2 launch robot_bringup game1.launch.py

# Game 2自動制御
ros2 launch robot_bringup game2_auto.launch.py
```

実機構成やCAN IDは `ros2_ws/src/robot_bringup/config/` で管理しています。実機へ接続する前に、対象機体の設定であることを確認してください。

## ドキュメント

- [ゲームパッド操作とjoy_controller](ros2_ws/src/joy_controller/README.md)
- [robot_controllerの概要](ros2_ws/src/robot_controller/README.md)
- [robot_bringupの起動構成](ros2_ws/src/robot_bringup/README.md)
- [hardware_driverの概要](ros2_ws/src/hardware_driver/README.md)
- [VESCドライバー](docs/hardware/vesc.md)
- [EduLite 05ドライバー](docs/hardware/edulite05.md)
- [STM32ドライバー](docs/hardware/stm32.md)
- [メカナムコントローラー](docs/controllers/mecanum.md)
- [ばねコントローラー](docs/controllers/spring.md)
- [ドリブルコントローラー](docs/controllers/dribble.md)
- [Game 2・PK自動照準](docs/controllers/game2-aim.md)
- [YOLOボール検出のセットアップ](docs/yolo_ball_setup.md)
- [RDK X5のセットアップ・検証スクリプト](script/README.md)
- [Raspberry Pi 5コントローラー転送](raspi_controller/RasberryPi5/README.md)
- [RDK X5コントローラー受信](raspi_controller/RDKX5/README.md)

## ディレクトリ構成

```text
.
├── docs/                  # 設計・機構・セットアップ資料
├── raspi_controller/      # Raspberry Pi向け補助処理
├── receiveCanConfiguration/
├── ros2_ws/               # ROS 2ワークスペース
│   └── src/               # ROS 2パッケージ
├── script/                # 解析・検証用スクリプト
├── docker-compose.yml
├── Dockerfile
└── Makefile
```

## 開発への参加

変更は作業ブランチで行い、ビルドと関連テストを通したうえでPull Requestを作成してください。ハードウェア依存の変更では、静的確認やビルド結果と、CANログ・実機確認の結果を区別して記録してください。

## ライセンス

本リポジトリは[MIT License](LICENSE)で公開されています。
