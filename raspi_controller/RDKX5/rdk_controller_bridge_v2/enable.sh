#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/controller/dualsense.yaml ]]; then
  echo "ERROR: /etc/controller/dualsense.yaml is missing" >&2
  exit 1
fi
sudo systemctl daemon-reload
sudo systemctl enable --now rdk-rfcomm.service
sudo systemctl enable --now controller-bridge.service

echo "Enabled RDK RFCOMM + controller bridge."
echo "Use remote-joy.service only if joy_node is not already in robot_bringup."
