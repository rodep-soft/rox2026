# 特殊ボール対応 YOLO 認識モデルの作成とデプロイガイド

手元にある物理ボール（色未定・試作品）から、本番で何色になっても高精度に認識できる YOLO モデルを作成し、RDK X5 (BPU) 上で動かす手順です。

---

## STEP 1. ボールの写真撮影 (30〜50枚)

手元にあるボールをスマホ等で撮影します。

### 撮影のコツ
1. **多様なアングル**: 上、斜め、横、離れた位置、近い位置。
2. **多様な背景**: 床、机、アームの近く、影のある場所。
3. **一部隠れた状態**: 壁や手で少しボールが隠れている写真も数枚混ぜる。

---

## STEP 2. Roboflow でのアノテーション＆色拡張エクスポート

1. 無料の [Roboflow](https://roboflow.com/) にサインインし、プロジェクト（Object Detection）を作成します。
2. 撮影した写真をドラッグ＆ドロップでアップロードします。
3. ボールを矩形枠で囲み、クラス名を `ball` に設定します。
4. **【最重要】Augmentation (データ拡張) の設定**:
   - `Generate` -> `Augmentation` を選択
   - **Hue (色相)**: `±50°` （これにより赤・青・黄・緑などの全色データが自動生成されます）
   - **Brightness (明るさ)**: `±25%`
   - **Exposure (露出)**: `±25%`
5. `Export` -> `YOLOv5 PyTorch` または `YOLOv8` 形式でダウンロード（または Colab 用 URL 取得）します。

---

## STEP 3. ローカル PC または Google Colab での学習

### A. ローカル PC で学習する場合 (Mac / Linux / Windows)

ローカルのターミナルでそのまま学習を実行できます。Mac (Apple Silicon) の場合は GPU (`device=mps`) 加速で高速に完了します。

```bash
# 1. ultralytics パッケージのインストール
pip install ultralytics

# 2. ローカル環境での学習実行 (Mac GPU指定: device=mps)
yolo detect train data=/path/to/dataset/data.yaml model=yolov8n.pt epochs=50 imgsz=640 device=mps

# 3. 完成したモデルを ONNX 形式へ変換
yolo export model=runs/detect/train/weights/best.pt format=onnx
```

---

### B. Google Colab (クラウド) で学習する場合 (約 5 分)

```python
# 1. ultralytics のインストール
!pip install ultralytics

# 2. 学習の実行
!yolo detect train data=data.yaml model=yolov8n.pt epochs=50 imgsz=640

# 3. ONNX 形式へのエクスポート
!yolo export model=runs/detect/train/weights/best.pt format=onnx
```

# 4. ONNX 形式へのエクスポート
!yolo export model=runs/detect/train/weights/best.pt format=onnx
```

学習完了後、生成された `best.onnx` (または `best.pt`) をダウンロードします。

---

## STEP 4. RDK X5 (BPU) へのデプロイ ＆ ROS 2 Launch 起動

ダウンロードしたモデルファイルを RDK X5 に配置し、追加済みの `yolo_launch.py` で起動します。

```bash
# 環境読み込み
source /opt/tros/humble/setup.bash
source ~/rox2026/ros2_ws/install/setup.bash

# YOLO ボール検出ノードの起動
ros2 launch robot_bringup yolo_launch.py

# 230AI ステレオビジョン ＋ YOLO ボール検出の同時起動
ros2 launch robot_bringup vision_launch.py enable_yolo:=true

# ロボット全機能 ＋ ビジョン ＋ AprilTag ＋ YOLO ボール検出の一括起動
ros2 launch robot_bringup robot.launch.py enable_vision:=true enable_apriltag:=true enable_yolo:=true
```

---

## 📊 検出結果の取得

YOLO が検出したボールのバウンディングボックス（画面上座標）および確信度は ROS 2 トピックから取得可能です。
