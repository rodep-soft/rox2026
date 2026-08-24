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
OUT_IMG_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/images"
OUT_LBL_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/labels"

os.makedirs(f"{OUT_IMG_DIR}/train", exist_ok=True)
os.makedirs(f"{OUT_IMG_DIR}/val", exist_ok=True)
os.makedirs(f"{OUT_LBL_DIR}/train", exist_ok=True)
os.makedirs(f"{OUT_LBL_DIR}/val", exist_ok=True)

def detect_ball_fast(img):
    # 高解像度画像を 640px にリサイズして超高速・高精度検出
    h, w = img.shape[:2]
    scale = 640.0 / max(h, w)
    small_w = int(w * scale)
    small_h = int(h * scale)
    small = cv2.resize(img, (small_w, small_h))
    
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    dilated = cv2.dilate(edges, kernel, iterations=2)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_box = None
    max_score = -1
    for c in contours:
        area = cv2.contourArea(c)
        if area > (small_w * small_h * 0.02):
            x, y, bw, bh = cv2.boundingRect(c)
            aspect = float(bw) / float(bh)
            if 0.65 <= aspect <= 1.5:
                # 中央寄りかつ適度なサイズのものを高スコア
                cx = x + bw / 2.0
                cy = y + bh / 2.0
                dist_center = np.hypot(cx - small_w/2, cy - small_h/2) / (small_w/2)
                score = area - dist_center * 500
                if score > max_score:
                    max_score = score
                    best_box = (int(x / scale), int(y / scale), int((x + bw) / scale), int((y + bh) / scale))
                    
    if best_box is not None:
        return best_box
    return int(w * 0.15), int(h * 0.15), int(w * 0.85), int(h * 0.85)

raw_files = [f for f in glob.glob(f"{RAW_DIR}/*.*") if not f.endswith(".txt") and not f.endswith(".yaml")]
print(f"🚀 Processing {len(raw_files)} raw images with ultra-fast vectorization...")

np.random.seed(42)
np.random.shuffle(raw_files)
split_idx = int(len(raw_files) * 0.85)

for idx, path in enumerate(raw_files):
    split = "train" if idx < split_idx else "val"
    try:
        pil_img = Image.open(path).convert('RGB')
        img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    except Exception:
        img = cv2.imread(path)
        
    if img is None:
        continue
    h, w = img.shape[:2]
    x1, y1, x2, y2 = detect_ball_fast(img)
    
    # 640px にリサイズして保存 (学習を爆速化)
    scale = 640.0 / max(h, w)
    img_640 = cv2.resize(img, (int(w * scale), int(h * scale)))
    
    box_cx = ((x1 + x2) / 2.0) / w
    box_cy = ((y1 + y2) / 2.0) / h
    box_w = max(0.05, min(1.0, (x2 - x1) / w))
    box_h = max(0.05, min(1.0, (y2 - y1) / h))
    
    base_name = os.path.splitext(os.path.basename(path))[0]
    cv2.imwrite(f"{OUT_IMG_DIR}/{split}/{base_name}.jpg", img_640)
    with open(f"{OUT_LBL_DIR}/{split}/{base_name}.txt", "w") as f:
        f.write(f"0 {box_cx:.6f} {box_cy:.6f} {box_w:.6f} {box_h:.6f}\n")

print(f"🎉 Auto-annotation done: {split_idx} train, {len(raw_files)-split_idx} val images prepared!")
