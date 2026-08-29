#!/usr/bin/env bash
set -Eeuo pipefail
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./restore-network-wait-online.sh" >&2
  exit 1
fi
systemctl unmask NetworkManager-wait-online.service || true
systemctl enable NetworkManager-wait-online.service || true
systemctl daemon-reload
echo "NetworkManager-wait-online.service restored. Reboot to compare boot behavior."
