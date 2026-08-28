# RDK X5 redundant controller receiver — fail-safe v2

This directory is the **complete RDK-side receiver** for a Raspberry Pi that
transmits one game-controller `evdev` stream redundantly over:

- Wi-Fi UDP (default port 5000)
- Bluetooth Classic RFCOMM (`/dev/rfcomm0`, channel 1)

The RDK exposes exactly one virtual game controller through Linux `uinput`.

## Required behavior

The bridge implements these states:

```text
valid packet on either path
        |
        v
     NORMAL
        |
        | both paths silent 200 ms
        v
     NEUTRAL       -> sends neutral axes + button releases once
        |
        | both paths silent 1.0 s total
        v
  DISCONNECTED     -> stops uinput -> /dev/input/eventX disappears
        |
        | any valid heartbeat/event returns
        v
     NORMAL         -> immediately recreates uinput, seeds neutral, accepts fresh events
```

A single live path is enough. Bluetooth can disappear while UDP continues and
vice versa. `rdk-rfcomm.service` and `controller-bridge.service` are deliberately
not hard-coupled, so a Bluetooth failure does not kill the UDP receiver.

## Why Interception Tools `uinput` is kept

The bridge does not reimplement the Linux virtual-input device capability
setup. Interception Tools can create a virtual input device from a YAML device
description and consume the native `struct input_event` stream on stdin.

The bridge owns that `uinput` subprocess so it can intentionally terminate it
after a 1-second total link loss. Closing/stopping the uinput creator removes
the corresponding virtual evdev device. When traffic returns, it starts a new
uinput process immediately.

## RDK dependencies

- Ubuntu / 64-bit aarch64 on RDK X5
- Python 3 (standard library only for `controller-bridge`)
- BlueZ `rfcomm`
- Interception Tools `uinput`
- ROS 2 Humble `joy` only if this RDK is also responsible for `joy_node`

The Linux kernel uinput module is loaded automatically by the installer.

## One-time controller description

The exact input capabilities must come from the real DualSense once. On the
Raspberry Pi, identify its joystick event node and run:

```bash
uinput -p -d /dev/input/by-id/<DualSense-event-joystick> > dualsense.yaml
```

Change the generated YAML `NAME:` to:

```yaml
NAME: Remote DualSense
```

Copy it to the RDK:

```bash
sudo mkdir -p /etc/controller
sudo install -m 0644 dualsense.yaml /etc/controller/dualsense.yaml
```

This is a one-time setup. Thereafter the RDK needs no manual operation.

## Neutral configuration

`neutral.json` included here uses the usual DualSense joystick-event mapping:

| evdev code | symbolic | neutral |
|---:|---|---:|
| 0 | ABS_X | 128 |
| 1 | ABS_Y | 128 |
| 2 | ABS_Z (L2) | 0 |
| 3 | ABS_RX | 128 |
| 4 | ABS_RY | 128 |
| 5 | ABS_RZ (R2) | 0 |
| 16 | ABS_HAT0X | 0 |
| 17 | ABS_HAT0Y | 0 |

All button codes that the bridge has seen are released automatically on the
200-ms fail-safe transition. Verify the axis values once with `evtest` on the
Raspberry Pi. If your kernel reports a different range/center, edit
`/etc/controller/neutral.json`; no program changes are needed.

## Install

```bash
chmod +x install.sh
./install.sh
```

After placing `/etc/controller/dualsense.yaml`:

```bash
./enable.sh
```

From then on, RDK boot automatically starts:

1. RDK/BlueZ Bluetooth services
2. RFCOMM channel 1 listener
3. redundant UDP + RFCOMM controller bridge

## ROS joy_node

Your existing `robot_bringup` already contains a `joy_node` in many robot
setups. If so, **do not enable `remote-joy.service`**. Set that launch node to:

```yaml
device_name: "Remote DualSense"
autorepeat_rate: 50.0
```

If nothing else starts joy_node, the included optional service can do it:

```bash
sudo systemctl enable --now remote-joy.service
```

ROS 2 `joy_node` uses SDL and supports a `device_name` parameter. The current
implementation also handles SDL joystick add/remove events; the bridge's
1-second uinput removal is therefore represented as an actual device removal.

## Sender protocol expected from Raspberry Pi

This v2 bridge intentionally adds a sender `session_id` so a Raspberry Pi reboot
can reset its sequence counter without old packets being confused with new
ones.

Each record is fixed-size:

```text
network byte order header: !2sBBII
  magic      2 B   "RC"
  version    1 B   2
  kind       1 B   0=event, 1=heartbeat
  session_id 4 B   random uint32 chosen once per sender process start
  seq        4 B   uint32 incremented once per event

payload:
  24 B native Linux struct input_event on aarch64
```

Total record size on the RDK/Raspberry Pi 64-bit ARM design: **36 bytes**.
Heartbeat records use a zero-filled 24-byte payload and do not increment the
event sequence. Send a heartbeat approximately every **50 ms (20 Hz)** over
both transports, even when the controller is untouched.

For UDP, a datagram may contain one or more whole records; never split a record
between UDP datagrams. RFCOMM is a byte stream; the RDK side handles framing
and resynchronization.

## Important recovery behavior

After a >1-second outage the virtual controller is recreated in **neutral**.
The bridge deliberately does not restore a stick/button that was held before
the outage. This prevents an old held-forward command from becoming active
again merely because the link returned. A fresh event from the controller then
updates the new virtual device normally.

## Logs / diagnosis

```bash
journalctl -u controller-bridge.service -f
journalctl -u rdk-rfcomm.service -f
```

Expected state transitions:

```text
state WAITING -> NORMAL
path udp: UP
path bluetooth: UP
state NORMAL -> NEUTRAL
FAILSAFE: neutral state emitted
state NEUTRAL -> DISCONNECTED
virtual controller destroyed
state DISCONNECTED -> NORMAL
creating virtual controller ...
```

Run the bundled diagnostic script:

```bash
./diagnose.sh
```

## Configuration

Edit `/etc/default/controller-bridge`:

```text
NEUTRAL_TIMEOUT=0.20
DISCONNECT_TIMEOUT=1.00
UDP_PORT=5000
```

Then:

```bash
sudo systemctl restart controller-bridge.service
```

## Safety note

This bridge provides a transport/input-device fail-safe, but motor-command
watchdogs should still exist in the downstream robot control nodes. Independent
layers of timeout protection are appropriate for a moving robot.
