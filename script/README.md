# `script/`

RDK X5の初期セットアップ、デバイス確認、動作試験、ログ解析などで使用するスクリプトをまとめています。

## ファイル一覧

### `rdk_setup.sh`

RDK X5の初期セットアップスクリプトです。ログインパスワード、Wi-Fi、有線LAN、APT・ROS 2パッケージ、SSH、Bluetooth、GitHub、Tailscale、CAN、rosdep、`.bashrc` などをまとめて設定します。

詳しい入力内容と実行手順は [`rdk-x5-setup.md`](./rdk-x5-setup.md) を参照してください。

```bash
cd ~
chmod +x rdk_setup.sh
./rdk_setup.sh
```

初期セットアップでは、USBメモリから `rdk_setup.sh` だけをホームディレクトリへコピーして実行します。リポジトリはスクリプトが `~/rox2026` へクローンします。`sudo` を付けて実行せず、スクリプト内で求められた場合にパスワードを入力します。

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
