# Raspberry Pi 5 redundant DualSense forwarder — RDK bridge v2 compatible

This is the complete Raspberry Pi side for the matching **RDK controller bridge v2**.
After one-time configuration/pairing, normal operation is fully automatic:

```text
Raspberry Pi power ON
  -> BlueZ starts
  -> raspi-rfcomm.service keeps trying to connect to the RDK
  -> controller-udevmon.service watches input hotplug
  -> DualSense appears
  -> udevmon starts intercept automatically
  -> controller-net-sender starts with a new session_id
  -> the same input_event records go to Wi-Fi UDP + Bluetooth RFCOMM
  -> heartbeat continues even while the controller is untouched

DualSense removed / its input device disappears
  -> intercept closes
  -> sender exits
  -> heartbeat stops
  -> RDK: 200 ms -> neutral
          1.0 s -> virtual event device removed

DualSense reconnects
  -> udevmon launches a new pipeline automatically
  -> immediate heartbeat
  -> RDK recreates the virtual controller immediately
```

No start command is required after boot once setup is complete.

## Design

The Raspberry Pi does **not** decode joystick axes/buttons into a custom controller
message. Interception Tools `intercept` reads the Linux evdev stream and writes raw
`struct input_event` records to stdout. The small `controller-net-sender` only adds:

- protocol magic/version
- sender session ID
- event sequence number
- heartbeat records

It sends the exact same record over both transports.

`udevmon` handles hotplug. The configuration matches a DualSense/Wireless Controller
node that has `BTN_SOUTH` and `ABS_X`, avoiding the separate motion-sensor/touchpad
event nodes.

## Runtime services

### `raspi-rfcomm.service`

Runs continuously from boot and executes BlueZ `rfcomm connect` to the RDK on
channel 1. If the RDK is off, Bluetooth is temporarily unavailable, or the link
drops, systemd retries every second forever. Boot order therefore does not matter.

### `controller-udevmon.service`

Runs `udevmon` continuously. When the physical DualSense gamepad event device
appears, it launches:

```text
intercept -g /dev/input/eventX | controller-net-sender
```

The `-g` can be disabled with `GRAB_DEVICE=0`, but a dedicated transmitter Pi should
normally keep it enabled.

## Protocol compatibility

Compatible with `rdk_controller_bridge_v2`:

```text
network header: !2sBBII (12 B)
  magic      "RC"
  version    2
  kind       0=event, 1=heartbeat
  session_id random uint32 per sender process
  seq        uint32 incremented once per input_event

payload: native 64-bit Linux struct input_event (24 B)
record total: 36 B
```

Heartbeat defaults to 50 ms (20 Hz) and is sent over both paths while the physical
controller pipeline is alive. Heartbeats do not increment `seq`.

Both Raspberry Pi and RDK are expected to use a 64-bit Linux ABI where
`struct input_event` is 24 bytes (`aarch64`, `getconf LONG_BIT` -> 64).

## Files

- `controller-net-sender` — protocol/UDP/RFCOMM sender
- `controller-forwarder-job` — `intercept` pipeline wrapper and retry
- `dualsense-forwarder.yaml` — udevmon hotplug match
- `controller-udevmon.service` — hotplug daemon at boot
- `raspi-rfcomm-run` — RFCOMM client wrapper
- `raspi-rfcomm.service` — persistent Bluetooth reconnect service
- `controller-forwarder.env` — all site-specific settings
- `install.sh` — dependencies + installation
- `pair-rdk.sh` — one-time Raspberry Pi -> RDK Bluetooth pair/trust helper
- `enable.sh` / `disable.sh` — boot enable/disable
- `make-dualsense-yaml.sh` — generate the RDK virtual-device YAML from real DualSense
- `diagnose.sh` — collect state/logs
- `test-udp-receiver.py` — bench protocol decoder

## 1. Install

On Raspberry Pi:

```bash
unzip raspi_controller_forwarder_v2.zip
cd raspi_controller_forwarder_v2
sudo ./install.sh
```

The installer installs BlueZ and Interception Tools dependencies. If
`interception-tools` is not available from the distribution package index, it builds
the upstream v0.5.0 tag and installs `intercept`, `udevmon`, `uinput`, and `mux` under
`/usr/local/bin`.

## 2. Set RDK addresses

Edit:

```bash
sudo nano /etc/default/controller-forwarder
```

Required values:

```text
RDK_HOST=10.82.0.120
RDK_BT_MAC=18:CE:DF:78:B6:FA
```

Use the real RDK Wi-Fi address and Bluetooth MAC. A fixed/DHCP-reserved Wi-Fi address
is recommended for a robot. `RDK_HOST` may also be a resolvable hostname.

Defaults:

```text
UDP_PORT=5000
WIFI_INTERFACE=wlan0
RFCOMM_CHANNEL=1
RFCOMM_DEV=/dev/rfcomm0
HEARTBEAT_INTERVAL=0.05
GRAB_DEVICE=1
```

If your Pi Wi-Fi interface is not `wlan0`, check `ip -br link` and change it.
Leaving `WIFI_INTERFACE=` empty allows normal Linux routing instead of forcing UDP to
one interface.

## 3. One-time Bluetooth pair/trust

On the RDK, temporarily make it pairable/discoverable and keep an agent running:

```bash
bluetoothctl
power on
agent on
default-agent
pairable on
discoverable on
```

Then on Raspberry Pi:

```bash
sudo ./pair-rdk.sh
```

Verify `Paired: yes` and `Trusted: yes` on both sides. On the RDK, trust the Raspberry
Pi once if necessary:

```text
trust <RASPI_BT_MAC>
discoverable off
```

This pairing/trust setup is one-time. Future boots use the stored BlueZ keys.

## 4. Enable fully automatic boot operation

```bash
sudo ./enable.sh
```

From the next boot onward, normally do nothing except power the Pi and RDK.

The RDK may boot first or the Raspberry Pi may boot first. `raspi-rfcomm.service`
continues retrying until the RDK listener becomes available.

## 5. Generate `dualsense.yaml` for the RDK

This is required once for the RDK `uinput` virtual controller definition.
Connect the DualSense to the Raspberry Pi, then:

```bash
sudo ./make-dualsense-yaml.sh dualsense.yaml
```

The helper finds the DualSense gamepad event node (not the touchpad/motion node),
runs Interception Tools `uinput -p -d`, and renames it `Remote DualSense`.

Copy the result to the RDK:

```bash
scp dualsense.yaml <RDK_USER>@<RDK_HOST>:/tmp/
```

On the RDK:

```bash
sudo install -m 0644 /tmp/dualsense.yaml /etc/controller/dualsense.yaml
sudo systemctl restart controller-bridge.service
```

## Normal status checks

```bash
systemctl status raspi-rfcomm.service
systemctl status controller-udevmon.service
rfcomm -a
```

Logs:

```bash
journalctl -u raspi-rfcomm.service -f
journalctl -u controller-udevmon.service -f
```

When the controller is attached you should see a job such as:

```text
[controller-forwarder-job] forwarding /dev/input/eventX
[controller-net-sender] start session=0x........
[controller-net-sender] UDP target resolved: ...
[controller-net-sender] Bluetooth RFCOMM writer connected: /dev/rfcomm0
```

Every 10 seconds the sender prints counters for events, heartbeats, UDP, and Bluetooth.

## Failure/recovery behavior

### Wi-Fi dies, Bluetooth lives

UDP sends fail, but the sender process continues and RFCOMM keeps transmitting.
The RDK continues using the Bluetooth copy.

### Bluetooth dies, Wi-Fi lives

The Bluetooth writer closes its fd and stops accepting Bluetooth records. UDP is
unaffected. `raspi-rfcomm.service` retries the RFCOMM connection independently. Once
`/dev/rfcomm0` comes back, the writer reopens it automatically and future records are
sent on both paths again.

### RDK is powered off

- UDP send errors are tolerated.
- RFCOMM service retries every second.
- controller event capture remains alive.
- when RDK returns, both transports recover without restarting the Pi.

### DualSense disconnects

`intercept` ends, the sender exits, and heartbeat stops. This is deliberate: the RDK
must treat controller loss as a real control-link loss instead of treating the Pi
itself being alive as controller health.

### DualSense reconnects

`udevmon` sees the device immediately and starts a new sender session. The first
heartbeat causes the RDK to recreate its virtual event device even if the user has not
moved a stick yet.

## Important Bluetooth queue behavior

The Bluetooth write path runs in its own thread, so a blocked/broken RFCOMM connection
cannot block UDP or event capture. Records are only queued while `/dev/rfcomm0` is
actually open. On write failure or queue overflow the queue is cleared rather than
replaying old controller actions after reconnection.

Missing sequence numbers are safe with the matching RDK v2 receiver: duplicate copies
are merged and a genuinely missing sequence is skipped after the RDK gap timeout.

## Diagnostics

Run:

```bash
sudo ./diagnose.sh
```

It shows:

- architecture/ABI
- network interfaces/routes
- Bluetooth controller state
- RFCOMM devices
- input device names
- forwarder processes
- both systemd services
- recent journal logs

## Bench test UDP without the RDK

On another Linux machine (same 64-bit input ABI):

```bash
python3 test-udp-receiver.py --port 5000
```

Point `RDK_HOST` at that machine and attach the controller. The decoder displays both
heartbeat and input-event records.

## Security / permissions

The hotplug service runs as root because:

- `intercept -g` needs access to `/dev/input/event*` and device grabbing
- `SO_BINDTODEVICE` may require privilege
- `/dev/rfcomm0` is commonly root/dialout restricted

The sender executes no network-provided commands and its payload is fixed-size binary
input-event data only.
