#!/usr/bin/env bash
set -Eeuo pipefail

echo "=== RDK X5 230AI MIPI Stereo Camera Check ==="

if ! command -v i2cdetect >/dev/null 2>&1; then
  echo "Installing i2c-tools..."
  sudo apt update && sudo apt install -y i2c-tools
fi

echo ""
echo "--- Scanning I2C Bus 4 (Left Camera / PHY0) ---"
sudo i2cdetect -r -y 4 2>/dev/null || echo "Bus 4 not available"

echo ""
echo "--- Scanning I2C Bus 6 (Right Camera / PHY2) ---"
sudo i2cdetect -r -y 6 2>/dev/null || echo "Bus 6 not available"

echo ""
echo "230AI Camera Expected Addresses:"
echo " - Expected I2C slave addresses: 0x30, 0x32, 0x50"
