# ROX Navigation

ROX 2026向けの自己位置推定・TF・Nav2自律走行パッケージです。ROS 2 Humbleを対象にしています。

## 構成

TFは次の1本の木になります。

```text
map --(ekf_map)--> odom --(ekf_odom)--> base_link
                                          |-- imu_link
                                          |-- camera_link
                                          |-- four wheel links
```

- `mecanum_odometry_node`: 4輪の実測角速度から `/wheel/odometry` を生成
- `can_imu_node`: `/socketcan_bridge/rx` のCANフレームを `/imu/data` に変換
- `tag_localization_node`: `/detection` と既知タグ座標から `/tag/pose` を生成
- `ekf_odom`: 車輪＋IMUを融合し、連続な `odom -> base_link` を生成
- `ekf_map`: ARタグ絶対位置を融合し、`map -> odom` を生成
- Nav2: NavFn（A*）＋ホロノミックDWB＋Behavior Tree

## ビルドと起動

```bash
cd ~/rox2026/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch rox_navigation autonomous.launch.py
```

ハードウェアドライバは別ターミナルで起動してください。

```bash
ros2 launch robot_bringup hardware.launch.py
ros2 launch robot_bringup mecanum_controller.launch.py
```

ゴール送信例:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

## IMUをCANで送る仕様

現在のデコーダは、little-endianの符号付き16 bit整数を次の3フレームで受けます。CAN IDと倍率は `config/sensors.yaml` で変更できます。

| CAN ID（既定） | byte 0..7 | 単位（変換後） |
|---|---|---|
| `0x500` | `ax, ay, az`（各int16、残り2 byte未使用） | m/s² |
| `0x501` | `gx, gy, gz`（各int16、残り2 byte未使用） | rad/s |
| `0x502` | `qx, qy, qz, qw`（各int16） | 正規化Quaternion |

既定倍率:

- 加速度: 1 count = 0.001 g = `0.00980665 m/s²`
- 角速度: 1 count = 0.001 deg/s = `1.74532925199433e-5 rad/s`
- Quaternion: 1 count = `1/16384`

最低限必要なのはZ軸角速度です。ただしドリフトを抑えるため、姿勢Quaternion（特にyaw）も推奨します。加速度は将来のスリップ検出や状態推定改善に利用できます。以下も実機仕様として確定してください。

1. IMU座標系: REP-103に従いX前、Y左、Z上
2. 送信周期: gyro/quaternionは50～100 Hz、accelは50 Hz以上を推奨
3. 全フレームの時間差: `max_component_age`（既定100 ms）以内
4. 起動時バイアス校正、温度補償、飽和値
5. CAN ID、endianness、倍率、欠落検出用sequence番号（必要ならプロトコル拡張）
6. `imu_link` の実際の取付位置・向きをURDFへ反映

## ARタグ `/detection` の仕様とTF

現在は次の入力を想定しています。

```text
Topic: /detection
Type: geometry_msgs/msg/PoseStamped
header.frame_id: camera_link（または実際の光学フレーム）
pose: カメラ座標系から見たタグの姿勢 camera -> tag
```

タグの地図上の既知姿勢 `map -> tag` と、URDFの `base_link -> camera_link` を使い、次式でロボット姿勢を計算します。

```text
map->base = map->tag * inverse(camera->tag) * inverse(base->camera)
```

したがって、検出結果を直接 `/tf` に流す必要はありません。`tag_localization_node` が `PoseWithCovarianceStamped` の `/tag/pose` に変換し、`ekf_map` が安定化した `map -> odom` TFをpublishします。検出ノードがAprilTag独自型やPoseArrayを出す場合は、タグIDとposeを取り出して `PoseStamped` に変換する小さなアダプタを追加してください。

調整必須項目:

- `config/sensors.yaml` の `tag_position` と `tag_rpy`
- URDFの `camera_joint` の位置・姿勢
- カメラが optical frame（X右、Y下、Z前）を使う場合、そのframeから`camera_link`への固定joint
- 複数タグを使う場合はタグIDごとの地図姿勢テーブルへ拡張

## 地図とURDF

`maps/arena.*` は5 m角の動作確認用地図です。実環境のPGM/YAMLへ置き換え、起動時に指定できます。

```bash
ros2 launch rox_navigation autonomous.launch.py map:=/absolute/path/to/map.yaml
```

`urdf/rox2026.urdf.xacro` は現在分かっている車体寸法による最小モデルです。実測した車体外形、IMU、カメラ位置に更新してください。

## 現在の制約

LiDARやdepth cameraが未接続のため、local costmapは動的障害物を観測しません。現在は静的地図上の障害物だけを避けます。人や移動物体がいる環境で実機走行させる前に、LaserScanまたはPointCloud2のobstacle/voxel layer、非常停止、速度制限を追加してください。