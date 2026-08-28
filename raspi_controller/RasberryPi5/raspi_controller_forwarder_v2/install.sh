#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./install.sh" >&2
  exit 1
fi

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y \
  bluez python3 git cmake build-essential \
  libevdev-dev libudev-dev libyaml-cpp-dev libboost-dev

install_interception_from_source() {
  local work=/tmp/interception-tools-build
  rm -rf "$work"
  git clone --depth 1 --branch v0.5.0 \
    https://gitlab.com/interception/linux/tools.git "$work"
  cmake -S "$work" -B "$work/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$work/build" --parallel "$(nproc)"

  local name src
  for name in intercept udevmon uinput mux; do
    src="$(find "$work/build" -type f -name "$name" -perm -111 | head -n1 || true)"
    if [[ -z "$src" ]]; then
      echo "Could not locate built $name" >&2
      exit 1
    fi
    install -m 0755 "$src" "/usr/local/bin/$name"
  done
  rm -rf "$work"
}

if ! command -v intercept >/dev/null 2>&1 || ! command -v udevmon >/dev/null 2>&1; then
  # Use a distro package when available; otherwise build the upstream v0.5.0 tag.
  if apt-cache show interception-tools >/dev/null 2>&1; then
    apt-get install -y interception-tools
  else
    install_interception_from_source
  fi
fi

for cmd in intercept udevmon rfcomm python3; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Required command missing after installation: $cmd" >&2
    exit 1
  fi
done

install -m 0755 "$HERE/controller-net-sender" /usr/local/bin/controller-net-sender
install -m 0755 "$HERE/controller-forwarder-job" /usr/local/bin/controller-forwarder-job
install -m 0755 "$HERE/controller-udevmon-run" /usr/local/bin/controller-udevmon-run
install -m 0755 "$HERE/raspi-rfcomm-run" /usr/local/bin/raspi-rfcomm-run

install -d -m 0755 /etc/interception
install -m 0644 "$HERE/dualsense-forwarder.yaml" /etc/interception/dualsense-forwarder.yaml

if [[ ! -e /etc/default/controller-forwarder ]]; then
  install -m 0644 "$HERE/controller-forwarder.env" /etc/default/controller-forwarder
  echo "Created /etc/default/controller-forwarder"
else
  echo "Keeping existing /etc/default/controller-forwarder"
fi

install -m 0644 "$HERE/controller-udevmon.service" /etc/systemd/system/controller-udevmon.service
install -m 0644 "$HERE/raspi-rfcomm.service" /etc/systemd/system/raspi-rfcomm.service

systemctl daemon-reload
systemctl enable bluetooth.service >/dev/null 2>&1 || true
systemctl start bluetooth.service >/dev/null 2>&1 || true

if systemctl is-active --quiet udevmon.service 2>/dev/null; then
  echo
  echo "WARNING: udevmon.service is already active."
  echo "This package uses controller-udevmon.service with its own config."
  echo "Do not run both if another config can grab the same DualSense device."
fi

echo
echo "Installation complete."
echo "1) Edit: sudo nano /etc/default/controller-forwarder"
echo "2) One-time Bluetooth pair/trust: sudo ./pair-rdk.sh"
echo "3) Enable automatic operation: sudo ./enable.sh"
