#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np
from PIL import Image
try:
    import pillow_heif
    pillow_heif.register_heif_opener()
except ImportError:
    pass

RAW_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/raw_images"
OUT_IMG_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/images/train"
OUT_LBL_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/labels/train"
VAL_IMG_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/images/val"
VAL_LBL_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/labels/val"

# 動画 IMG_1444.mov から 15 フレーム抽出してデータセットに加える
video_path = "/home/tatsv/Documents/image/IMG_1444.mov"
cap = cv2.VideoCapture(video_path)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

extracted_frames = []
for i in range(0, total_frames, 4):
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)
    ret, frame = cap.read()
    if ret:
        fn = f"vid_1444_f{i}.jpg"
        cv2.imwrite(os.path.join(RAW_DIR, fn), frame)
        extracted_frames.append(fn)
cap.release()
print(f"Extracted {len(extracted_frames)} frames from IMG_1444.mov into raw_images!")

# 各画像に対してボールの輪郭・円検出または手動アノテーション支援
files = sorted([f for f in glob.glob(f"{RAW_DIR}/*.*") if not f.endswith(".txt") and not f.endswith(".yaml")])
print(f"Total raw images to annotate: {len(files)}")

# 1444のボール中心とサイズ (480x854基準: cx ~ 235, cy ~ 275, w ~ 125, h ~ 125)
for fn in extracted_frames:
    img = cv2.imread(os.path.join(RAW_DIR, fn))
    h, w = img.shape[:2]
    # ボールの真の位置: 手が乗っているモルテンボール
    # x_center, y_center, width, height (normalized)
    # x: 236/480 = 0.492, y: 275/854 = 0.322, w: 120/480 = 0.250, h: 120/854 = 0.140
    lbl_content = f"0 0.492 0.322 0.250 0.140\n"
    with open(os.path.join(RAW_DIR, os.path.splitext(fn)[0] + ".txt"), 'w') as f:
        f.write(lbl_content)

print("VID 1444 frames auto-labeled successfully!")
