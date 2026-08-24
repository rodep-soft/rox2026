#!/usr/bin/env python3
import os
import glob
import shutil
import cv2
import numpy as np
from PIL import Image
import pillow_heif
pillow_heif.register_heif_opener()

DATA_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset"
RAW_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/raw_images"

shutil.rmtree(DATA_DIR, ignore_errors=True)
os.makedirs(f"{DATA_DIR}/images/train", exist_ok=True)
os.makedirs(f"{DATA_DIR}/labels/train", exist_ok=True)
os.makedirs(f"{DATA_DIR}/images/val", exist_ok=True)
os.makedirs(f"{DATA_DIR}/labels/val", exist_ok=True)

# 1. 写真とラベルをリサイズして格納
files = sorted([f for f in glob.glob(f"{RAW_DIR}/*.*") if not f.endswith(".txt") and not f.endswith(".yaml")])

for idx, p in enumerate(files):
    base = os.path.splitext(os.path.basename(p))[0]
    txt_path = f"{RAW_DIR}/{base}.txt"
    if not os.path.exists(txt_path):
        continue
    
    with open(txt_path, 'r') as f:
        lbl_content = f.read().strip()
    if not lbl_content:
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
    out_img = f"{DATA_DIR}/images/{split}/{base}.jpg"
    out_lbl = f"{DATA_DIR}/labels/{split}/{base}.txt"
    
    cv2.imwrite(out_img, img_640)
    with open(out_lbl, 'w') as f:
        f.write(lbl_content + "\n")

# 2. ネガティブ背景画像の注入 (4枚)
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

print(f"Dataset generated:")
print(f"  Train: {len(os.listdir(f'{DATA_DIR}/images/train'))} images")
print(f"  Val: {len(os.listdir(f'{DATA_DIR}/images/val'))} images")
