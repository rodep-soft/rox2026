# Raspberry Pi 5 DualSense redundant forwarder — final v3

DualSense connected to the Raspberry Pi is forwarded to the RDK X5 over both:

- Wi-Fi UDP -> `10.42.0.251:5000`
- Bluetooth Classic RFCOMM -> RDK `18:CE:DF:78:E6:56`, channel 1

The two paths carry the same protocol-v2 sequence/session IDs so the RDK can deduplicate them.

## Final tuned settings

`/etc/default/controller-forwarder` is installed with:

```ini
AXIS_RATE=120
ABS_MIN_DELTA=1
BT_QUEUE_SIZE=8
BT_MAX_LATENCY=0.03
HEARTBEAT_INTERVAL=0.05
```

`AXIS_RATE=120` is a maximum analog report rate, not a mandatory 120-Hz stream. Unchanged axes are suppressed. Button transitions remain urgent. The 20-Hz heartbeat is independent and is used for link/fail-safe state.

## Install

```bash
cd raspi_controller_forwarder_final_v3
sudo ./install.sh
```

The installer now handles all known missing-dependency cases from the test setup:

- BlueZ / `rfcomm`
- Python 3
- Interception Tools
- Debian/Raspberry Pi OS trixie `interception-tools-compat` (restores upstream `intercept` command)
- automatic Interception Tools v0.6.8 source-build fallback
- the two helper scripts that were accidentally missing from the older v2.1 ZIP:
  - `controller-udevmon-run`
  - `raspi-rfcomm-run`

The installer backs up an existing `/etc/default/controller-forwarder` and installs the final tuned configuration.


## Fast boot change

This final version includes only the boot optimization that was verified on the Raspberry Pi:

- disables and masks `NetworkManager-wait-online.service`
- removes `network-online.target` from `controller-udevmon.service`
- leaves `NetworkManager.service` enabled
- leaves all other system services unchanged
- keeps RFCOMM dependent only on `bluetooth.service`

The forwarding process is designed to start before Wi-Fi is ready. UDP begins working when `wlan0` becomes available, while RFCOMM reconnects independently. On the tested Pi this change reduced total boot time from about 12.7 s to about 6.6 s.

To undo only this optimization later:

```bash
sudo ./restore-network-wait-online.sh
```

## One-time Bluetooth pairing/trust

If this Pi is not already paired/trusted with the RDK:

```bash
sudo ./pair-rdk.sh
```

Then enable automatic operation:

```bash
sudo ./enable.sh
```

After that, normal boot behavior is automatic:

1. BlueZ starts.
2. `raspi-rfcomm.service` keeps trying to establish `/dev/rfcomm0` to the RDK.
3. `controller-udevmon.service` watches for the DualSense joystick event node.
4. Hotplug of the DualSense starts `intercept -g ... | controller-net-sender`.
5. Unplug stops the real controller stream and therefore stops heartbeat/event delivery.

## Check

```bash
systemctl status raspi-rfcomm.service controller-udevmon.service
journalctl -u controller-udevmon.service -f
```

A healthy active sender shows arguments including:

```text
--axis-rate 120
--bt-queue-size 8
--bt-max-latency 0.03
--heartbeat-interval 0.05
```

Check directly with:

```bash
pgrep -af controller-net-sender
```

Healthy sender stats should normally keep:

```text
bt_connected=True
bt_qdrop=0
bt_stale=0
```

`reports=` is the useful counter for estimating controller report frequency. The raw `sent=`/RDK `bt=` counters are input-event records and can be several times larger than the report rate.

## Wi-Fi-only / Bluetooth-only test

To test Bluetooth only:

```bash
sudo ip link set wlan0 down
```

Do this only if you have another way to access the Pi; SSH over `wlan0` will disconnect. Restore with:

```bash
sudo ip link set wlan0 up
```

## Diagnostics

```bash
sudo ./diagnose.sh
```

