#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

need=(python3 rfcomm)
for cmd in "${need[@]}"; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: required command not found: $cmd" >&2
    exit 1
  fi
done

UINPUT_SRC="$(command -v uinput || true)"
if [[ -z "$UINPUT_SRC" ]]; then
  cat >&2 <<'EOF'
ERROR: Interception Tools 'uinput' is not installed.
Install Interception Tools first, then rerun this installer.
EOF
  exit 1
fi

echo "Using Interception Tools uinput: $UINPUT_SRC"

sudo install -m 0755 "$HERE/controller-bridge" /usr/local/bin/controller-bridge
sudo install -m 0755 "$HERE/controller-bridge-run" /usr/local/bin/controller-bridge-run
sudo install -m 0755 "$HERE/joy-remote-run" /usr/local/bin/joy-remote-run

# Pin/copy the uinput executable to a stable path used by the service.
if [[ "$UINPUT_SRC" != "/usr/local/bin/uinput" ]]; then
  sudo install -m 0755 "$UINPUT_SRC" /usr/local/bin/uinput
fi

sudo install -m 0644 "$HERE/rdk-rfcomm.service" /etc/systemd/system/rdk-rfcomm.service
sudo install -m 0644 "$HERE/controller-bridge.service" /etc/systemd/system/controller-bridge.service
sudo install -m 0644 "$HERE/remote-joy.service" /etc/systemd/system/remote-joy.service

sudo install -m 0644 "$HERE/controller-bridge.env" /etc/default/controller-bridge
sudo install -m 0644 "$HERE/remote-joy.env" /etc/default/remote-joy

sudo mkdir -p /etc/controller
sudo install -m 0644 "$HERE/neutral.json" /etc/controller/neutral.json

echo uinput | sudo tee /etc/modules-load.d/uinput.conf >/dev/null
sudo modprobe uinput
sudo systemctl daemon-reload

cat <<'EOF'

RDK bridge programs installed.

ONE-TIME remaining step:
  Install the controller device description generated from the physical
  DualSense on the Raspberry Pi as:
      /etc/controller/dualsense.yaml

Then enable the always-on receiver:
  sudo systemctl enable --now rdk-rfcomm.service
  sudo systemctl enable --now controller-bridge.service

If joy_node is NOT already launched by your robot_bringup launch file, also run:
  sudo systemctl enable --now remote-joy.service

If robot_bringup already starts joy_node, DO NOT enable remote-joy.service.
Configure that existing joy_node with:
  device_name: "Remote DualSense"
  autorepeat_rate: 50.0
EOF
