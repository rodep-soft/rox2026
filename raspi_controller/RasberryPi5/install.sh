#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./install.sh" >&2
  exit 1
fi
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y bluez python3 iproute2 git cmake build-essential \
  libevdev-dev libudev-dev libyaml-cpp-dev libboost-dev

valid_intercept() { command -v intercept >/dev/null 2>&1 && intercept -h 2>&1 | grep -q 'redirect device input events'; }
valid_udevmon()  { command -v udevmon  >/dev/null 2>&1 && udevmon  -h 2>&1 | grep -q 'monitor input devices'; }

install_interception_from_source() {
  local work=/tmp/interception-tools-build
  echo "Installing Interception Tools v0.6.8 from upstream source..."
  rm -rf "$work"
  git clone --depth 1 --branch v0.6.8 https://gitlab.com/interception/linux/tools.git "$work"
  cmake -S "$work" -B "$work/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$work/build" --parallel "$(nproc)"
  local name src
  for name in intercept udevmon uinput mux; do
    src="$(find "$work/build" -type f -name "$name" -perm -111 | head -n1 || true)"
    [[ -n "$src" ]] || { echo "Could not locate built $name" >&2; exit 1; }
    install -m 0755 "$src" "/usr/local/bin/$name"
  done
  rm -rf "$work"
}

# Raspberry Pi OS/Debian trixie provides Interception Tools, but upstream
# `intercept` is split into interception-tools-compat. Install both when available.
if ! valid_intercept || ! valid_udevmon; then
  if apt-cache show interception-tools >/dev/null 2>&1; then
    apt-get install -y interception-tools
    if apt-cache show interception-tools-compat >/dev/null 2>&1; then
      apt-get install -y interception-tools-compat
    fi
  fi
fi

if ! valid_intercept || ! valid_udevmon; then
  install_interception_from_source
fi

for cmd in intercept udevmon rfcomm python3; do
  command -v "$cmd" >/dev/null 2>&1 || { echo "Required command missing: $cmd" >&2; exit 1; }
done

install -m 0755 "$HERE/controller-net-sender" /usr/local/bin/controller-net-sender
install -m 0755 "$HERE/controller-forwarder-job" /usr/local/bin/controller-forwarder-job
install -m 0755 "$HERE/controller-udevmon-run" /usr/local/bin/controller-udevmon-run
install -m 0755 "$HERE/raspi-rfcomm-run" /usr/local/bin/raspi-rfcomm-run

install -d -m 0755 /etc/interception
install -m 0644 "$HERE/dualsense-forwarder.yaml" /etc/interception/dualsense-forwarder.yaml

if [[ -e /etc/default/controller-forwarder ]]; then
  cp -a /etc/default/controller-forwarder "/etc/default/controller-forwarder.bak.$(date +%Y%m%d-%H%M%S)"
fi
install -m 0644 "$HERE/controller-forwarder.env" /etc/default/controller-forwarder

install -m 0644 "$HERE/controller-udevmon.service" /etc/systemd/system/controller-udevmon.service
install -m 0644 "$HERE/raspi-rfcomm.service" /etc/systemd/system/raspi-rfcomm.service

systemctl daemon-reload

# Fast boot: this controller forwarder does not need to block boot until the
# network is fully online. UDP reconnects automatically and RFCOMM is managed
# independently by raspi-rfcomm.service.
if systemctl list-unit-files NetworkManager-wait-online.service >/dev/null 2>&1; then
  systemctl disable NetworkManager-wait-online.service >/dev/null 2>&1 || true
  systemctl mask NetworkManager-wait-online.service >/dev/null 2>&1 || true
fi

systemctl enable bluetooth.service >/dev/null 2>&1 || true
systemctl start bluetooth.service >/dev/null 2>&1 || true

echo
echo "Raspberry Pi installation complete."
echo "Final settings: AXIS_RATE=120, BT_QUEUE_SIZE=8, BT_MAX_LATENCY=0.03, heartbeat=20Hz"
echo "Fast boot: NetworkManager-wait-online.service is disabled/masked; controller forwarding does not wait for network-online.target"
echo "Current target: RDK 10.42.0.251 / BT 18:CE:DF:78:E6:56"
echo
echo "First setup only:"
echo "  sudo ./pair-rdk.sh"
echo "  sudo ./enable.sh"
echo
echo "If already paired/trusted, just run: sudo ./enable.sh"
