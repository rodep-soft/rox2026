#!/bin/bash
# ROX2026 Emergency Reset & Diagnostics Script

echo "--- ROX2026 Reset & Diagnostics ---"

echo "[1/3] Clearing FastDDS shared memory..."
sudo rm -rf /dev/shm/fastrtps_*

echo "[2/3] Re-initializing SocketCAN (can0)..."
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100

echo "[3/3] Running CAN node diagnostics..."
python3 $(dirname "$0")/can_health_check.py

