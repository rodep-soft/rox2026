#!/usr/bin/env python3
import os
import glob
import cv2
from PIL import Image
try:
    import pillow_heif
    pillow_heif.register_heif_opener()
except ImportError:
    pass
from ultralytics import YOLO

model = YOLO("/home/tatsv/rox2026/yolo_ball_pipeline/models/molten_ball_best.pt")
raw_dir = "/home/tatsv/rox2026/yolo_ball_pipeline/raw_images"
out_dir = "/home/tatsv/.gemini/antigravity-cli/brain/77ad1587-f00a-4587-b34d-b08015e7f5e9/eval_results"
os.makedirs(out_dir, exist_ok=True)

files = [f for f in glob.glob(f"{raw_dir}/*.*") if not f.endswith(".txt") and not f.endswith(".yaml")]
print(f"Testing {len(files)} photos...")

detected = 0
for path in files:
    try:
        pil_img = Image.open(path).convert('RGB')
        import numpy as np
        img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    except Exception:
        img = cv2.imread(path)
    if img is None:
        continue
        
    results = model(img, conf=0.25, verbose=False)
    has_ball = False
    for r in results:
        if len(r.boxes) > 0:
            has_ball = True
            annotated = r.plot()
            base = os.path.splitext(os.path.basename(path))[0]
            cv2.imwrite(f"{out_dir}/res_{base}.jpg", annotated)
    if has_ball:
        detected += 1

print(f"\n==========================================")
print(f"🎉 FINAL STATS: {detected} / {len(files)} ({detected/len(files)*100:.1f}%) BALLS DETECTED!")
print(f"==========================================")
