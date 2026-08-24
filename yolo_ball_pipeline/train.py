#!/usr/bin/env python3
import os
from ultralytics import YOLO

def train_ball_detector(epochs=40, model_size='n', batch_size=8):
    yaml_path = "/home/tatsv/rox2026/yolo_ball_pipeline/ball_dataset.yaml"
    model_weight = f"yolov8{model_size}.pt"
    
    print(f"🚀 Starting YOLOv8 ({model_weight}) Training for Ball Detection...")
    model = YOLO(model_weight)
    
    results = model.train(
        data=yaml_path,
        epochs=epochs,
        imgsz=640,
        batch=batch_size,
        workers=4,
        project="/home/tatsv/rox2026/yolo_ball_pipeline/runs",
        name="molten_ball",
        exist_ok=True,
        plots=True,
        augment=True, # 色・回転・拡大縮小の自動データ拡張
    )
    
    best_model_path = "/home/tatsv/rox2026/yolo_ball_pipeline/runs/molten_ball/weights/best.pt"
    deploy_path = "/home/tatsv/rox2026/yolo_ball_pipeline/models/molten_ball_best.pt"
    
    if os.path.exists(best_model_path):
        import shutil
        shutil.copy(best_model_path, deploy_path)
        print(f"\n🎉 Training Succeeded! Best Model saved to: {deploy_path}")
        
        # ONNX 出力 (RDK X5 / BPU 変換用)
        try:
            print("📦 Exporting model to ONNX format...")
            best_model = YOLO(best_model_path)
            best_model.export(format="onnx", imgsz=640)
            print("✅ ONNX Model successfully exported!")
        except Exception as e:
            print(f"⚠️ ONNX export failed: {e}")

if __name__ == '__main__':
    train_ball_detector()
