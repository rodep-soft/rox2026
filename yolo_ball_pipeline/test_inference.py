#!/usr/bin/env python3
import os
import glob
import cv2
from ultralytics import YOLO

def test_inference(test_dir="/home/tatsv/rox2026/yolo_ball_pipeline/raw_images"):
    model_path = "/home/tatsv/rox2026/yolo_ball_pipeline/models/molten_ball_best.pt"
    if not os.path.exists(model_path):
        model_path = "yolov8n.pt"
        print("⚠️ Custom best.pt not found yet. Using base yolov8n.pt...")
    else:
        print(f"🎯 Loading Trained Ball Model: {model_path}")
        
    model = YOLO(model_path)
    out_dir = "/home/tatsv/rox2026/yolo_ball_pipeline/runs/test_results"
    os.makedirs(out_dir, exist_ok=True)
    
    images = glob.glob(f"{test_dir}/*.*")
    print(f"🔍 Testing {len(images)} images from {test_dir}...")
    
    for img_path in images:
        img = cv2.imread(img_path)
        if img is None:
            continue
        results = model(img, conf=0.25, verbose=False)
        for r in results:
            for box in r.boxes:
                cls_name = model.names[int(box.cls[0].item())]
                conf = float(box.conf[0].item())
                xyxy = [round(x, 1) for x in box.xyxy[0].tolist()]
                print(f"  [{os.path.basename(img_path)}] -> {cls_name} ({conf*100:.1f}%) @ {xyxy}")
            
            annotated = r.plot()
            out_file = f"{out_dir}/res_{os.path.basename(img_path)}"
            cv2.imwrite(out_file, annotated)

    print(f"\n✅ All results saved to: {out_dir}")

if __name__ == '__main__':
    test_inference()
