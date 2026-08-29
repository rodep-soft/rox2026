#!/usr/bin/env bash
set +e

echo '===== system ====='
uname -a
printf 'arch: '; uname -m
printf 'long bits: '; getconf LONG_BIT

echo
echo '===== config ====='
if [[ -r /etc/default/controller-forwarder ]]; then
  sed -E 's/(RDK_BT_MAC=).*/\1<configured>/' /etc/default/controller-forwarder
else
  echo 'missing /etc/default/controller-forwarder'
fi

echo
echo '===== network ====='
ip -br addr
ip route

echo
echo '===== bluetooth ====='
bluetoothctl show
rfcomm -a
ls -l /dev/rfcomm* 2>/dev/null

echo
echo '===== input devices ====='
for e in /sys/class/input/event*; do
  [[ -r "$e/device/name" ]] || continue
  printf '%-12s %s\n' "/dev/input/$(basename "$e")" "$(cat "$e/device/name")"
done

echo
echo '===== processes ====='
pgrep -a -f 'udevmon|intercept|controller-net-sender|rfcomm' || true

echo
echo '===== services ====='
systemctl --no-pager --full status bluetooth.service raspi-rfcomm.service controller-udevmon.service

echo
echo '===== recent RFCOMM logs ====='
journalctl -u raspi-rfcomm.service -n 40 --no-pager

echo
echo '===== recent forwarder logs ====='
journalctl -u controller-udevmon.service -n 80 --no-pager
