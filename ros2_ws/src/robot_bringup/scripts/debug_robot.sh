#!/bin/bash
# ロボット一発復帰＆デバッグスクリプト

echo "========================================="
echo " 🛠️ ROX2026 自動トラブルシューティング 🛠️"
echo "========================================="

echo "[1/3] FastDDS 共有メモリの残骸ロックを削除中..."
sudo rm -rf /dev/shm/fastrtps_*

echo "[2/3] SocketCAN (can0) を再初期化中..."
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100

echo "[3/3] 全モータ＆CANノードの生滅チェックを実行します..."
python3 $(dirname "$0")/can_health_check.py
