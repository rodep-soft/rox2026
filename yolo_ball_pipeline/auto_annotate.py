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

def load_image_any(path):
    try:
        pil_img = Image.open(path).convert('RGB')
        return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    except Exception as e:
        return cv2.imread(path)

def detect_ball_hybrid(img):
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 1.5)
    
    # 1. ハフ円検出
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=h/5,
        param1=80, param2=35, minRadius=int(min(w,h)*0.08), maxRadius=int(min(w,h)*0.48)
    )
    if circles is not None:
        circles = np.uint16(np.around(circles))
        best = circles[0][0]
        cx, cy, r = int(best[0]), int(best[1]), int(best[2])
        return max(0, cx - r), max(0, cy - r), min(w, cx + r), min(h, cy + r)
        
    # 2. 輪郭検出
    edges = cv2.Canny(blurred, 40, 120)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    dilated = cv2.dilate(edges, kernel, iterations=2)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area > (w * h * 0.03):
            x, y, bw, bh = cv2.boundingRect(c)
            aspect = float(bw) / float(bh)
            if 0.7 <= aspect <= 1.4:
                candidates.append((area, x, y, x + bw, y + bh))
                
    if candidates:
        candidates.sort(reverse=True, key=lambda x: x[0])
        _, x1, y1, x2, y2 = candidates[0]
        return x1, y1, x2, y2

    # 3. フォールバック
    return int(w * 0.15), int(h * 0.15), int(w * 0.85), int(h * 0.85)

def process_dataset(val_ratio=0.15):
    raw_files = [f for f in glob.glob(f"{RAW_DIR}/*.*") if not f.endswith(".txt") and not f.endswith(".yaml")]
    print(f"🔍 Found {len(raw_files)} raw ball images in {RAW_DIR}")
    if not raw_files:
        return

    np.random.seed(42)
    np.random.shuffle(raw_files)
    split_idx = int(len(raw_files) * (1 - val_ratio))

    for idx, path in enumerate(raw_files):
        split = "train" if idx < split_idx else "val"
        img = load_image_any(path)
        if img is None:
            continue
        h, w = img.shape[:2]
        x1, y1, x2, y2 = detect_ball_hybrid(img)
        
        box_cx = ((x1 + x2) / 2.0) / w
        box_cy = ((y1 + y2) / 2.0) / h
        box_w = max(0.05, (x2 - x1) / w)
        box_h = max(0.05, (y2 - y1) / h)
        
        base_name = os.path.splitext(os.path.basename(path))[0]
        out_img_path = f"{OUT_IMG_DIR}/{split}/{base_name}.jpg"
        out_lbl_path = f"{OUT_LBL_DIR}/{split}/{base_name}.txt"
        
        cv2.imwrite(out_img_path, img)
        with open(out_lbl_path, "w") as f:
            f.write(f"0 {box_cx:.6f} {box_cy:.6f} {box_w:.6f} {box_h:.6f}\n")
            
    print(f"🎉 Prepared dataset: {split_idx} train images, {len(raw_files)-split_idx} validation images.")

if __name__ == '__main__':
    process_dataset()
