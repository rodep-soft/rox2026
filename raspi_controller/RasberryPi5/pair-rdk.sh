#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./pair-rdk.sh" >&2
  exit 1
fi
ENV_FILE=/etc/default/controller-forwarder
# shellcheck disable=SC1090
source "$ENV_FILE"
: "${RDK_BT_MAC:?Set RDK_BT_MAC in $ENV_FILE}"
if [[ "$RDK_BT_MAC" == "CHANGE_ME" ]]; then
  echo "Set RDK_BT_MAC in $ENV_FILE first." >&2
  exit 2
fi

systemctl enable --now bluetooth.service

echo "RDK must be pairable/discoverable for this one-time step."
echo "Starting pairing with $RDK_BT_MAC ..."

# Keep the agent and the pair command inside the same bluetoothctl process.
# pair waits for the pairing transaction before the following trust command.
timeout 60 bluetoothctl <<BT
power on
agent on
default-agent
scan on
pair $RDK_BT_MAC
trust $RDK_BT_MAC
scan off
info $RDK_BT_MAC
quit
BT

echo
echo "Raspberry Pi side pairing/trust command completed."
echo "Verify above that Paired: yes and Trusted: yes are shown."
echo "Also trust this Raspberry Pi once on the RDK side if it is not already Trusted=yes."
