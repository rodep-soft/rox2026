#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./enable.sh" >&2
  exit 1
fi

ENV_FILE=/etc/default/controller-forwarder
if [[ ! -r "$ENV_FILE" ]]; then
  echo "Missing $ENV_FILE. Run install.sh first." >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ENV_FILE"

if [[ "${RDK_HOST:-CHANGE_ME}" == "CHANGE_ME" || -z "${RDK_HOST:-}" ]]; then
  echo "Set RDK_HOST in $ENV_FILE first." >&2
  exit 2
fi
if [[ "${RDK_BT_MAC:-CHANGE_ME}" == "CHANGE_ME" || -z "${RDK_BT_MAC:-}" ]]; then
  echo "Set RDK_BT_MAC in $ENV_FILE first." >&2
  exit 2
fi

if [[ -n "${WIFI_INTERFACE:-}" ]] && ! ip link show "$WIFI_INTERFACE" >/dev/null 2>&1; then
  echo "WARNING: Wi-Fi interface '$WIFI_INTERFACE' does not currently exist." >&2
fi

systemctl daemon-reload
systemctl enable --now bluetooth.service
systemctl enable --now raspi-rfcomm.service
systemctl enable --now controller-udevmon.service

echo
echo "Enabled. From the next boot, no manual start command is required."
echo "RFCOMM may show auto-restart until the RDK is powered and listening; that is intentional."
echo
echo "Check logs:"
echo "  journalctl -u raspi-rfcomm.service -f"
echo "  journalctl -u controller-udevmon.service -f"
