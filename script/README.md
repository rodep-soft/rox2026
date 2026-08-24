# `script/`

RDK X5の初期セットアップ、デバイス確認、動作試験、ログ解析などで使用するスクリプトをまとめています。

## ファイル一覧

### `rdk_setup.sh`

RDK X5の初期セットアップスクリプトです。ログインパスワード、Wi-Fi、有線LAN、APT・ROS 2パッケージ、SSH、Bluetooth、GitHub、Tailscale、CAN、rosdep、`.bashrc` などをまとめて設定します。

詳しい入力内容と実行手順は [`rdk-x5-setup.md`](./rdk-x5-setup.md) を参照してください。

```bash
chmod +x script/rdk_setup.sh
./script/rdk_setup.sh
```

`sudo` を付けて実行しないでください。スクリプトの途中で必要に応じて `sudo` のパスワードが求められます。

### `rdk-x5-setup.md`

RDK X5のセットアップ手順書です。OSイメージの書き込み、`rdk_setup.sh` の実行、Wi-Fi・ネットワーク設定、カメラ、USB Webカメラの確認方法を説明しています。

### `check_camera.sh`

230AI MIPIステレオカメラの接続確認スクリプトです。I2C Bus 4とBus 6をスキャンし、カメラで使用する `0x30`、`0x32`、`0x50` のアドレスを確認します。`i2c-tools` が未インストールの場合は自動でインストールします。

```bash
./script/check_camera.sh
```

### `pair_dualsense.sh`

DualSenseコントローラーをBluetoothでペアリングするスクリプトです。Bluetooth関連パッケージと `expect` を確認し、周辺のDualSenseをスキャンして、指定したMACアドレスに対してペアリング・信頼登録・接続を行います。最後に接続先MACアドレスを保存します。

実行前にDualSenseをペアリングモードにしてください（PSボタンとSHAREボタンを約3秒長押し）。

```bash
bash script/pair_dualsense.sh
```

候補のMACアドレスを選ぶか、MACアドレスを直接入力します。次回以降は保存されたアドレスを利用して再接続できます。

### `analyze_dribble_current.py`

ドリブルローラーの電流ログを解析するPythonスクリプトです。rosbag2の `.db3`、rosbagディレクトリ、CSV・テキストログを読み込み、電流分布のヒストグラム、二峰性、境界値、信頼度、ローパスフィルタの影響を分析します。最後に、ドリブルコントローラー用の推奨値と `ros2 param set` コマンドを出力します。

入力ファイルを省略すると、カレントディレクトリ以下から対象ログを自動検出します。

```bash
python3 script/analyze_dribble_current.py <ログまたはrosbag>

# logical_id とLPF係数を指定する場合
python3 script/analyze_dribble_current.py <ログ> --lid 12 --alpha 0.07
```

### `test_game1_wp_move.py`

Game1のwaypoint移動を確認するROS 2テストノードです。`/odometry/filtered` を購読し、指定した相対目標位置・姿勢へ移動するための速度指令を `/drive/cmd_vel` に出力します。移動完了時は `/game1/wp_test/completed` に `Bool` をpublishします。

ROS 2環境をsourceした状態で実行してください。

```bash
python3 script/test_game1_wp_move.py
```

目標位置、速度上限、許容誤差、タイムアウトなどは、スクリプト内でROS 2パラメータとして定義されています。

## 注意

- ネットワーク設定やパッケージのインストールを行うスクリプトは、RDK X5上で実行してください。
- Bluetoothやカメラの確認では、対象デバイスを事前に接続・ペアリングモードにしてください。
- ログ解析やROS 2テストでは、入力ログの場所とROS 2環境のsource状態を確認してください。
