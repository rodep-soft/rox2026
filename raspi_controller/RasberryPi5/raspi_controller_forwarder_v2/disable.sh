#!/usr/bin/env bash
set -Eeuo pipefail
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./disable.sh" >&2
  exit 1
fi
systemctl disable --now controller-udevmon.service 2>/dev/null || true
systemctl disable --now raspi-rfcomm.service 2>/dev/null || true
rfcomm release 0 >/dev/null 2>&1 || true
echo "Controller forwarding disabled."
