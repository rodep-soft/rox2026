#!/usr/bin/env bash
set -Eeuo pipefail
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./enable.sh" >&2
  exit 1
fi
[[ -r /etc/controller/dualsense.yaml ]] || { echo "Missing /etc/controller/dualsense.yaml" >&2; exit 1; }
systemctl daemon-reload
systemctl enable --now rdk-rfcomm.service
systemctl enable --now controller-bridge.service

echo "Enabled RDK RFCOMM + redundant controller bridge."
echo "The bridge only creates the Remote DualSense input device; ROS joy_node is not managed by this package."
