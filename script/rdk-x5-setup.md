# RDK X5のセットアップ


## 0. RDK X5の"Install Operating System"を見る
[RDK X5(Install OS) Docs](https://developer.d-robotics.cc/rdk_doc/en/Quick_start/install_os/rdk_x5/system_burn)
===
## 1. イメージをSDカードに書き込む
### 1.1 RufusのDownload
* [RufusのDownloadリンク](https://rufus.ie/en/)から"rufus-4.14.exe"を取ってくる
* その後ダウンロードしたファイルを実行

## 1.2. イメージのダウンロード
(一応記述するがDocsを見れはよろしい)
* [ImageをDownloadできるリンク](https://archive.d-robotics.cc/downloads/os_images/rdk_x5/rdk_os_3.5.0-2026-4-9/)に飛ぶ
* Ubuntu22.04のDesktopをDownloadしてくる

## 1.3. イメージをSDカードに書き込む
* SDカードを差し込み、Rufusを起動
* SDカードを選択し、Imageを選択し書き込む

## 2. SSHサービスの有効化
(もともと"SSHサービス"自体有効化されていたが一応記述する)
* 下に記述した"System"は左上の "application"を押すと出てきたはず <!--確認が必要-->
* System -> RDK Configuration -> Interface Options -> SSHでOn, Off可能

## 3. VNC(Vertual Network Computing)
: SSH越しで相手のPCのGUIを操作可能
* System -> RDK Configuration -> Interface Options -> VSCでOn, Off可能
(*デフォルトでPasswordは設定してないのでLogin Passwordを8字いないで作る必要がある)

## 4. 230AI MIPI ステレオカメラ (hobot_stereonet) のセットアップ

### 4.1. パッケージのインストール・更新
```bash
sudo apt update
sudo apt install -y tros-humble-hobot-stereonet tros-humble-mipi-cam ros-humble-apriltag-ros
```

### 4.2. 230AI カメラの認識チェック (I2C)
```bash
./script/check_camera.sh
```
* **230AI カメラ**: Bus 4 および Bus 6 で アドレス `0x30`, `0x32`, `0x50` が検出されれば正常接続です。

### 4.3. ビジョン ＆ AprilTag ノードの起動 (ROS 2 Launch)
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

### 4.4. 動作確認
* **Web UI 可視化**: ブラウザから `http://<RDK_IP>:8000` にアクセスし、RGB画像および深度マップが表示されることを確認。
* **AprilTag 認識データ**: `ros2 topic echo /tf` や `ros2 topic echo /detections` で検出した AprilTag の位置姿勢データを確認。
* **点群確認 (RViz2)**: PC 上の RViz2 で `/StereoNetNode/stereonet_pointcloud2` または AprilTag TF 座標枠を参照。



