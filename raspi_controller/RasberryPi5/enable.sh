#!/usr/bin/env bash
set -Eeuo pipefail
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./enable.sh" >&2
  exit 1
fi
ENV_FILE=/etc/default/controller-forwarder
[[ -r "$ENV_FILE" ]] || { echo "Missing $ENV_FILE. Run install.sh first." >&2; exit 1; }
# shellcheck disable=SC1090
source "$ENV_FILE"
: "${RDK_HOST:?Set RDK_HOST in $ENV_FILE}"
: "${RDK_BT_MAC:?Set RDK_BT_MAC in $ENV_FILE}"

systemctl daemon-reload
systemctl enable --now bluetooth.service
systemctl enable --now raspi-rfcomm.service
systemctl enable --now controller-udevmon.service

echo "Enabled Raspberry Pi controller forwarding."
echo "Check:"
echo "  systemctl status raspi-rfcomm.service controller-udevmon.service"
echo "  journalctl -u controller-udevmon.service -f"
