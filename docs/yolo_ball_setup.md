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

## STEP 3. Google Colab での無料高速学習 (約 5 分)

Google Colab（無料 GPU）でモデルを学習させます。

```python
# 1. ultralytics / YOLOv8 のインストール
!pip install ultralytics

# 2. Roboflow からデータセットのダウンロード (取得したコードを貼り付け)
# !curl -L "https://universe.roboflow.com/..."

# 3. 学習の実行 (色変化に強いナノモデル YOLOv8n / YOLOv5s)
!yolo detect train data=data.yaml model=yolov8n.pt epochs=50 imgsz=640

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
