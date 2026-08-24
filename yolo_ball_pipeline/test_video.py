#!/usr/bin/env python3
import os
import time
import cv2
from ultralytics import YOLO

input_video = "/home/tatsv/Documents/image/IMG_1447.mov"
model_path = "/home/tatsv/rox2026/yolo_ball_pipeline/models/molten_ball_best.pt"
out_dir = "/home/tatsv/.gemini/antigravity-cli/brain/77ad1587-f00a-4587-b34d-b08015e7f5e9/video_eval"
os.makedirs(out_dir, exist_ok=True)
out_video = os.path.join(out_dir, "tracked_IMG_1447.mp4")

if not os.path.exists(input_video):
    print(f"❌ Input video not found: {input_video}")
    exit(1)

print(f"🎬 Loading video: {input_video}")
cap = cv2.VideoCapture(input_video)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

print(f"🎥 Video Info: {width}x{height} @ {fps:.1f}fps, total {total_frames} frames")
print(f"🧠 Loading YOLO model: {model_path}")
model = YOLO(model_path)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(out_video, fourcc, fps, (width, height))

detected_frames = 0
frame_idx = 0
start_time = time.time()

sample_saved = False
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    frame_idx += 1
    
    # 推論 (conf=0.18)
    results = model(frame, conf=0.18, verbose=False)
    annotated = results[0].plot()
    
    boxes = results[0].boxes
    if len(boxes) > 0:
        detected_frames += 1
        if not sample_saved and frame_idx % 15 == 0:
            cv2.imwrite(os.path.join(out_dir, f"sample_frame_{frame_idx}.jpg"), annotated)
            sample_saved = True
            
    out.write(annotated)
    if frame_idx % 30 == 0:
        print(f"  Frame {frame_idx}/{total_frames} processed ({detected_frames/frame_idx*100:.1f}% tracking rate)...")

cap.release()
out.release()
elapsed = time.time() - start_time

print(f"\n==========================================")
print(f"🎉 VIDEO TRACKING EVALUATION COMPLETE!")
print(f"📊 Total Frames: {frame_idx}")
print(f"🎯 Detected & Tracked Frames: {detected_frames} ({detected_frames/max(1,frame_idx)*100:.1f}%)")
print(f"⚡ Processing Speed: {frame_idx/max(1e-3, elapsed):.1f} FPS (Elapsed: {elapsed:.2f}s)")
print(f"💾 Annotated Video Output: {out_video}")
print(f"==========================================")
