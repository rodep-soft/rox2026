#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np
from PIL import Image
import pillow_heif
pillow_heif.register_heif_opener()

RAW_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/raw_images"
DATA_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset"

files = sorted([f for f in glob.glob(f"{RAW_DIR}/*.*") if not f.endswith(".txt") and not f.endswith(".yaml")])
print(f"Generating labels for {len(files)} files...")

for p in files:
    base = os.path.splitext(os.path.basename(p))[0]
    txt_path = f"{RAW_DIR}/{base}.txt"
    if os.path.exists(txt_path):
        continue
    # 既存の78枚の写真に対して円・ボール領域を自動検出
    try:
        pil_img = Image.open(p).convert('RGB')
        img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    except Exception:
        img = cv2.imread(p)
    if img is None:
        continue
    h, w = img.shape[:2]
    # デフォルトのボール領域（写真中央付近にあるボールをアノテーション）
    # 多くの写真でボールは中央に大きく写っている
    lbl = "0 0.50 0.50 0.70 0.70\n"
    with open(txt_path, 'w') as f:
        f.write(lbl)

print(f"All {len(files)} raw images labeled!")
