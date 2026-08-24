#!/usr/bin/env python3
import os
import shutil
import cv2
import numpy as np
from PIL import Image
try:
    import pillow_heif
    pillow_heif.register_heif_opener()
except ImportError:
    pass

IMG_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/images/train"
LBL_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/labels/train"

neg_images = [
    "/home/tatsv/Documents/image/IMG_1457.webp",
    "/home/tatsv/Documents/image/download (1).png",
    "/home/tatsv/Documents/image/IMG_1451.webp",
    "/home/tatsv/Documents/image/IMG_1460.webp"
]

print("Adding negative background samples to dataset...")
for path in neg_images:
    if not os.path.exists(path):
        continue
    base = os.path.splitext(os.path.basename(path))[0]
    out_img = f"{IMG_DIR}/neg_{base}.jpg"
    out_lbl = f"{LBL_DIR}/neg_{base}.txt"
    
    pil_img = Image.open(path).convert('RGB')
    img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    h, w = img.shape[:2]
    scale = 640.0 / max(h, w)
    img_640 = cv2.resize(img, (int(w * scale), int(h * scale)))
    
    cv2.imwrite(out_img, img_640)
    # 空のラベルファイルを作成 (これがYOLOにおけるネガティブ背景の正攻法定義)
    open(out_lbl, 'w').close()
    print(f"  + Added negative: neg_{base}.jpg (empty label)")

print("Negative dataset injection complete!")
