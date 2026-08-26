# RDK X5のセットアップ

## 0. `rdk_setup.sh` による初期セットアップ

RDK X5 のOSを書き込んだ後、RoDEP環境をまとめてセットアップする場合は、リポジトリのルートで次のスクリプトを実行します。

```bash
cd ~/rox2026
chmod +x script/rdk_setup.sh
./script/rdk_setup.sh
```

### 実行時の注意

- スクリプト自体を `sudo` で実行しないでください。必要な処理ではスクリプト内部から `sudo` を使用します。
- インターネット接続が必要です。APTパッケージのインストール、GitHub認証、リポジトリ取得、Tailscaleのインストールを行います。
- 有線LAN `eth0` は `192.168.127.10/24` の固定アドレスに設定されます。既存の `eth0-static` 接続があれば更新されます。
- 処理の途中でエラーが発生した場合は、その時点で終了します。再実行時は、完了済みの設定を確認してから実行してください。

### 実行中に入力する内容

スクリプトを実行すると、基本的に次の順番で入力を求められます。

1. `Change login/SSH password now? [y/N]:`
   - `y` を入力して、ログインパスワードを新しく登録します。
   - `passwd` が起動したら、現在のパスワードを入力した後、新しいパスワードとして `rodeprodep` を入力し、確認でも同じパスワードを入力します。
2. `RoDEP-5Ghz Wi-Fi Password:`
   - Wi-Fiパスワードとして `rodeprodep` を入力します。
   - パスワードは画面に表示されません。入力後、Enterを押します。
   - Wi-FiのGUI設定で `Keep ...` の確認が表示された場合は、既定の選択を変更せず、そのまま Enter を押します。
3. `sudo` のパスワード入力
   - パッケージのインストールやネットワーク設定などで求められた場合、現在のユーザーのログインパスワードを入力します。
4. GitHub CLIの認証（未認証の場合のみ）
   - `gh auth login` の案内で、GitHub.com、SSH接続（`SSH`）、既存の公開鍵（`/home/<ユーザー名>/.ssh/id_ed25519.pub`）を選択します。
   - GitHubアカウントは `rodepshare@gmail.com` を使用します。
   - GitHubへのSSH接続確認に失敗すると、スクリプトはそこで終了します。
5. `Authenticate Tailscale now? [y/N]:`
   - このRDKがまだTailscaleに接続されていない場合だけ表示されます。
   - `y` を入力し、表示されたTailscaleの認証URLを開きます。
   - `rodepshare@gmail.com` に紐づくGitHubアカウントでログインして、Tailscaleの認証を完了します。

パスワード入力中は、入力内容が画面に表示されないことがあります。
`rodeprodep` は初期設定用のパスワードとして扱い、セットアップ完了後に変更してください。

### スクリプトが行う処理

1. ログインパスワードの変更（選択時のみ）
2. `RoDEP-5Ghz` Wi-Fiへの接続
3. `eth0-static` 有線LAN接続の作成または更新
4. 基本開発ツール、ROS 2 Humble関連パッケージのインストール
   - `ros-humble-v4l2-camera`
   - `ros-humble-apriltag-ros`
   - `ros-humble-tf2-ros` の再インストール
5. SSH・Bluetoothサービスの有効化と起動
6. GitHub CLIの認証確認、`main-v2` ブランチのクローン
7. Tailscaleのインストールとサービス起動
8. CANインターフェース `can0` の設定
   - bitrate: `1000000`
   - restart-ms: `100`
   - txqueuelen: `1024`
9. ROS 2 Humbleの環境確認と `rosdep` の初期化・更新
10. `.bashrc` へのROS 2、ccache、CANエイリアスの追加
11. `~/rox2026/ros2_ws/src` が存在する場合の依存パッケージ導入

完了後、現在のシェルに設定を反映します。

```bash
source ~/.bashrc
```

## 1. RDK X5の"Install Operating System"を見る
[RDK X5(Install OS) Docs](https://developer.d-robotics.cc/rdk_doc/en/Quick_start/install_os/rdk_x5/system_burn)
===
## 2. イメージをSDカードに書き込む
### 2.1 RufusのDownload
* [RufusのDownloadリンク](https://rufus.ie/en/)から"rufus-4.14.exe"を取ってくる
* その後ダウンロードしたファイルを実行

## 2.2. イメージのダウンロード
(一応記述するがDocsを見れはよろしい)
* [ImageをDownloadできるリンク](https://archive.d-robotics.cc/downloads/os_images/rdk_x5/rdk_os_3.5.0-2026-4-9/)に飛ぶ
* Ubuntu22.04のDesktopをDownloadしてくる

## 2.3. イメージをSDカードに書き込む
* SDカードを差し込み、Rufusを起動
* SDカードを選択し、Imageを選択し書き込む

## 3. SSHサービスの有効化
(もともと"SSHサービス"自体有効化されていたが一応記述する)
* 下に記述した"System"は左上の "application"を押すと出てきたはず <!--確認が必要-->
* System -> RDK Configuration -> Interface Options -> SSHでOn, Off可能

## 4. VNC(Vertual Network Computing)
: SSH越しで相手のPCのGUIを操作可能
* System -> RDK Configuration -> Interface Options -> VSCでOn, Off可能
(*デフォルトでPasswordは設定してないのでLogin Passwordを8字いないで作る必要がある)

## 5. 230AI MIPI ステレオカメラ (hobot_stereonet) のセットアップ

### 5.1. パッケージのインストール・更新
```bash
sudo apt update
sudo apt install -y tros-humble-hobot-stereonet tros-humble-mipi-cam ros-humble-apriltag-ros
```

### 5.2. 230AI カメラの認識チェック (I2C)
```bash
./script/check_camera.sh
```
* **230AI カメラ**: Bus 4 および Bus 6 で アドレス `0x30`, `0x32`, `0x50` が検出されれば正常接続です。

### 5.3. ビジョン ＆ AprilTag ノードの起動 (ROS 2 Launch)
```bash
source /opt/tros/humble/setup.bash
source ~/rox2026/ros2_ws/install/setup.bash

# 230AI 単体ビジョン起動
ros2 launch robot_bringup vision_launch.py

# 230AI ビジョン ＋ AprilTag 検出を同時に起動 (tag36h11, 16cmタグ)
ros2 launch robot_bringup vision_launch.py enable_apriltag:=true tag_family:=tag36h11 tag_size:=0.16

# ロボット全体起動 ＋ ビジョン ＋ AprilTag 同時起動
ros2 launch robot_bringup robot.launch.py enable_vision:=true enable_apriltag:=true
```

### 5.4. 動作確認
* **Web UI 可視化**: ブラウザから `http://<RDK_IP>:8000` にアクセスし、RGB画像および深度マップが表示されることを確認。
* **AprilTag 認識データ**: `ros2 topic echo /tf` や `ros2 topic echo /detections` で検出した AprilTag の位置姿勢データを確認。
* **点群確認 (RViz2)**: PC 上の RViz2 で `/StereoNetNode/stereonet_pointcloud2` または AprilTag TF 座標枠を参照。

## 6. USB Webカメラ (V4L2) のセットアップ

### 6.1. パッケージのインストール
```bash
sudo apt update
sudo apt install -y v4l-utils ros-humble-v4l2-camera
```

### 6.2. デバイス確認
```bash
v4l2-ctl --list-devices
```
`/dev/video0` などのデバイスノードが存在することを確認します。

### 6.3. ROS 2 Launch 起動
```bash
# USB Webカメラ単体起動 (/dev/video0)
ros2 launch robot_bringup webcam_launch.py video_device:=/dev/video0

# USB Webカメラ ＋ AprilTag 検出を同時に起動
ros2 launch robot_bringup webcam_launch.py video_device:=/dev/video0 enable_apriltag:=true

# ロボット全体起動と同時に Webカメラを起動
ros2 launch robot_bringup robot.launch.py enable_webcam:=true video_device:=/dev/video0
```
配信トピック: `/webcam/image_raw` (`sensor_msgs/msg/Image`), `/webcam/camera_info`
