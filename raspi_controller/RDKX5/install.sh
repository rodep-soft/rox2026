#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo ./install.sh" >&2
  exit 1
fi
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y bluez python3 git cmake build-essential \
  libevdev-dev libudev-dev libyaml-cpp-dev libboost-dev

valid_uinput() {
  command -v uinput >/dev/null 2>&1 && uinput -h 2>&1 | grep -q 'redirect device input events'
}

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

# Try distro packages first. Ubuntu images may not provide them, so source build
# is the automatic fallback instead of aborting the installation.
if ! valid_uinput; then
  if apt-cache show interception-tools >/dev/null 2>&1; then
    apt-get install -y interception-tools
  fi
fi
if ! valid_uinput; then
  install_interception_from_source
fi
valid_uinput || { echo "Interception Tools uinput installation failed" >&2; exit 1; }

# Pin the verified Interception Tools uinput at the stable service path.
UINPUT_SRC="$(command -v uinput)"
if [[ "$UINPUT_SRC" != /usr/local/bin/uinput ]]; then
  install -m 0755 "$UINPUT_SRC" /usr/local/bin/uinput
fi

install -m 0755 "$HERE/controller-bridge" /usr/local/bin/controller-bridge
install -m 0755 "$HERE/controller-bridge-run" /usr/local/bin/controller-bridge-run

install -m 0644 "$HERE/rdk-rfcomm.service" /etc/systemd/system/rdk-rfcomm.service
install -m 0644 "$HERE/controller-bridge.service" /etc/systemd/system/controller-bridge.service

for f in controller-bridge; do
  if [[ -e "/etc/default/$f" ]]; then
    cp -a "/etc/default/$f" "/etc/default/$f.bak.$(date +%Y%m%d-%H%M%S)"
  fi
done
install -m 0644 "$HERE/controller-bridge.env" /etc/default/controller-bridge

install -d -m 0755 /etc/controller
install -m 0644 "$HERE/neutral.json" /etc/controller/neutral.json
# This YAML is the DualSense description captured during the working setup.
# Preserve a locally modified file, otherwise install the known-good one.
if [[ ! -e /etc/controller/dualsense.yaml ]]; then
  install -m 0644 "$HERE/dualsense.yaml" /etc/controller/dualsense.yaml
else
  echo "Keeping existing /etc/controller/dualsense.yaml"
fi

echo uinput > /etc/modules-load.d/uinput.conf
modprobe uinput
[[ -c /dev/uinput ]] || { echo "/dev/uinput was not created after modprobe" >&2; exit 1; }

# Remove optional ROS joy helpers from an earlier package revision, if present.
# This package intentionally stops at the Linux virtual input device.
for svc in remote-joy.service remote-joy-throttle.service; do
  systemctl disable --now "$svc" >/dev/null 2>&1 || true
  rm -f "/etc/systemd/system/$svc"
done
rm -f /usr/local/bin/joy-remote-run /usr/local/bin/joy-throttle-run
rm -f /etc/default/remote-joy

systemctl daemon-reload

echo
echo "RDK installation complete."
echo "Interception Tools uinput: $(command -v uinput)"
echo "Fail-safe: neutral after 0.20 s, virtual device removal after 5.00 s."
echo "RFCOMM listener: channel 1."
echo
echo "Enable transport/virtual-controller services: sudo ./enable.sh"
echo "ROS joy_node is intentionally not installed or managed by this package."
