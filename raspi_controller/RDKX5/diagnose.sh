#!/usr/bin/env bash
set +e

echo '===== architecture ====='
uname -a
getconf LONG_BIT

echo '===== bluetooth ====='
bluetoothctl show 2>&1
ls -l /sys/class/bluetooth/ 2>&1

echo '===== RFCOMM ====='
rfcomm -a 2>&1
ls -l /dev/rfcomm* 2>&1

echo '===== Interception/uinput ====='
ls -l /dev/uinput 2>&1
command -v uinput 2>&1
uinput -h 2>&1 | head -8

echo '===== config ====='
cat /etc/default/controller-bridge 2>&1
ls -l /etc/controller/dualsense.yaml /etc/controller/neutral.json 2>&1

echo '===== services ====='
systemctl --no-pager --full status rdk-rfcomm.service controller-bridge.service 2>&1

echo '===== recent bridge log ====='
journalctl -u controller-bridge.service -n 80 --no-pager 2>&1

echo '===== recent rfcomm log ====='
journalctl -u rdk-rfcomm.service -n 50 --no-pager 2>&1

echo '===== input devices ====='
for e in /sys/class/input/event*; do
  [[ -r "$e/device/name" ]] || continue
  printf '%-12s %s\n' "/dev/input/$(basename "$e")" "$(cat "$e/device/name")"
done
