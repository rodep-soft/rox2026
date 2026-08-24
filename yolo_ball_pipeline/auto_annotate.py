#!/usr/bin/env python3
import os
import glob
import cv2
import numpy as np

RAW_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/raw_images"
OUT_IMG_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/images"
OUT_LBL_DIR = "/home/tatsv/rox2026/yolo_ball_pipeline/dataset/labels"

def detect_ball_heuristic(img):
    """
    ボールの幾何学パターン（白黒リブ・エッジ・円形性）から正確なバウンディングボックスを抽出
    """
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # 円形ハフ変換 ＋ 輪郭抽出
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=h/4,
        param1=100, param2=30, minRadius=int(h*0.1), maxRadius=int(h*0.48)
    )
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        best_circle = circles[0][0] # (x, y, r)
        cx, cy, r = int(best_circle[0]), int(best_circle[1]), int(best_circle[2])
        x1 = max(0, cx - r)
        y1 = max(0, cy - r)
        x2 = min(w, cx + r)
        y2 = min(h, cy + r)
        return (x1, y1, x2, y2)
    
    # フォールバック: コントラストエッジ検出
    edges = cv2.Canny(blurred, 30, 100)
    contours, _ = cv2.findContours(edges, cv2.REQU_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > (w * h * 0.05):
            x, y, bw, bh = cv2.boundingRect(largest)
            return (x, y, x + bw, y + bh)
            
    # 中央寄りデフォルト
    return (int(w*0.2), int(h*0.2), int(w*0.8), int(h*0.8))

def process_dataset(val_ratio=0.2):
    raw_files = glob.glob(f"{RAW_DIR}/*.*")
    print(f"🔍 Found {len(raw_files)} raw images in {RAW_DIR}")
    if not raw_files:
        print("⚠️ No raw images found! Place photos into yolo_ball_pipeline/raw_images/")
        return

    np.random.seed(42)
    np.random.shuffle(raw_files)
    split_idx = int(len(raw_files) * (1 - val_ratio))

    for idx, path in enumerate(raw_files):
        split = "train" if idx < split_idx else "val"
        img = cv2.imread(path)
        if img is None:
            continue
        h, w = img.shape[:2]
        x1, y1, x2, y2 = detect_ball_heuristic(img)
        
        # YOLO format (class x_center y_center width height) normalized 0..1
        box_cx = ((x1 + x2) / 2.0) / w
        box_cy = ((y1 + y2) / 2.0) / h
        box_w = (x2 - x1) / w
        box_h = (y2 - y1) / h
        
        base_name = os.path.splitext(os.path.basename(path))[0]
        out_img_path = f"{OUT_IMG_DIR}/{split}/{base_name}.jpg"
        out_lbl_path = f"{OUT_LBL_DIR}/{split}/{base_name}.txt"
        
        cv2.imwrite(out_img_path, img)
        with open(out_lbl_path, "w") as f:
            f.write(f"0 {box_cx:.6f} {box_cy:.6f} {box_w:.6f} {box_h:.6f}\n")
            
        print(f"✅ [{split.upper()}] Processed {base_name} -> Box: ({box_cx:.2f}, {box_cy:.2f}, {box_w:.2f}, {box_h:.2f})")

    print(f"\n🎉 Dataset successfully prepared: {split_idx} train, {len(raw_files)-split_idx} val images.")

if __name__ == '__main__':
    process_dataset()
