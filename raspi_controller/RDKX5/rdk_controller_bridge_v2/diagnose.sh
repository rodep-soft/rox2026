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

echo '===== uinput ====='
ls -l /dev/uinput 2>&1
command -v uinput 2>&1

echo '===== config ====='
ls -l /etc/controller/dualsense.yaml /etc/controller/neutral.json 2>&1

echo '===== services ====='
systemctl --no-pager --full status rdk-rfcomm.service 2>&1
systemctl --no-pager --full status controller-bridge.service 2>&1

echo '===== recent bridge log ====='
journalctl -u controller-bridge.service -n 80 --no-pager 2>&1

echo '===== recent rfcomm log ====='
journalctl -u rdk-rfcomm.service -n 50 --no-pager 2>&1

echo '===== input devices ====='
ls -l /dev/input/ 2>&1
if command -v ros2 >/dev/null 2>&1; then
  echo '===== ROS joy devices ====='
  bash -lc 'source /opt/ros/humble/setup.bash && ros2 run joy joy_enumerate_devices' 2>&1
fi
