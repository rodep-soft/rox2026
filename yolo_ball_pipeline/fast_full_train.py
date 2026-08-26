#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np
from PIL import Image
import pillow_heif
pillow_heif.register_heif_opener()

DATA_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset"
RAW_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/raw_images"

os.makedirs(f"{DATA_DIR}/images/train", exist_ok=True)
os.makedirs(f"{DATA_DIR}/labels/train", exist_ok=True)
os.makedirs(f"{DATA_DIR}/images/val", exist_ok=True)
os.makedirs(f"{DATA_DIR}/labels/val", exist_ok=True)

# 1. 1444動画からのフレームにボールラベルを即座に付与
# (480x854のボール位置: 手が乗っているボール x:0.492, y:0.322, w:0.250, h:0.140)
# (奥の白いボール x:0.56, y:0.07, w:0.12, h:0.07)
# (右奥の白赤ボール x:0.82, y:0.13, w:0.16, h:0.09)
for vf in glob.glob(f"{RAW_DIR}/vid_1444_*.jpg"):
    base = os.path.splitext(os.path.basename(vf))[0]
    with open(f"{RAW_DIR}/{base}.txt", 'w') as f:
        f.write("0 0.492 0.322 0.250 0.140\n")
        f.write("0 0.560 0.070 0.120 0.070\n")
        f.write("0 0.820 0.130 0.160 0.090\n")

# 2. 全画像(93枚)とラベルをリサイズして train / val に同期
files = sorted([f for f in glob.glob(f"{RAW_DIR}/*.*") if not f.endswith(".txt") and not f.endswith(".yaml")])
print(f"Syncing {len(files)} dataset files...")

for idx, p in enumerate(files):
    base = os.path.splitext(os.path.basename(p))[0]
    txt_path = f"{RAW_DIR}/{base}.txt"
    if not os.path.exists(txt_path):
        continue
    with open(txt_path, 'r') as f:
        lbl = f.read().strip()
    if not lbl:
        continue
    try:
        pil_img = Image.open(p).convert('RGB')
        img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    except Exception:
        img = cv2.imread(p)
    if img is None:
        continue
    h, w = img.shape[:2]
    scale = 640.0 / max(h, w)
    img_640 = cv2.resize(img, (int(w * scale), int(h * scale)))
    
    split = "val" if (idx % 7 == 0) else "train"
    cv2.imwrite(f"{DATA_DIR}/images/{split}/{base}.jpg", img_640)
    with open(f"{DATA_DIR}/labels/{split}/{base}.txt", 'w') as f:
        f.write(lbl + "\n")

# 3. ネガティブ背景画像の注入 (4枚)
neg_images = [
    "/home/tatsv/Documents/image/IMG_1457.webp",
    "/home/tatsv/Documents/image/download (1).png",
    "/home/tatsv/Documents/image/IMG_1451.webp",
    "/home/tatsv/Documents/image/IMG_1460.webp"
]
for path in neg_images:
    if not os.path.exists(path):
        continue
    base = os.path.splitext(os.path.basename(path))[0]
    pil_img = Image.open(path).convert('RGB')
    img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    h, w = img.shape[:2]
    scale = 640.0 / max(h, w)
    img_640 = cv2.resize(img, (int(w * scale), int(h * scale)))
    cv2.imwrite(f"{DATA_DIR}/images/train/neg_{base}.jpg", img_640)
    open(f"{DATA_DIR}/labels/train/neg_{base}.txt", 'w').close()

print(f"Data ready: Train={len(os.listdir(f'{DATA_DIR}/images/train'))}, Val={len(os.listdir(f'{DATA_DIR}/images/val'))}")
