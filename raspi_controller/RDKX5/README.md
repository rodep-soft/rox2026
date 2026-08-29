# RDK X5 redundant DualSense receiver — final v2

Receives the same DualSense `evdev` stream over Wi-Fi UDP and Bluetooth RFCOMM, deduplicates it, and creates one Linux virtual input device named `Remote DualSense` through Interception Tools `uinput`.

This package intentionally stops at the Linux input-device layer. It does **not** install, start, configure, throttle, or otherwise manage ROS 2 `joy_node` or `topic_tools`.

## Final fail-safe behavior

```text
Either path receives valid heartbeat/event
              -> NORMAL

Both paths silent >= 0.20 s
              -> NEUTRAL
                 neutral axes + button releases are emitted once

Both paths silent >= 5.00 s
              -> DISCONNECTED
                 virtual uinput device is destroyed

Any valid path returns
              -> virtual device recreated immediately -> NORMAL
```

The 5-second removal delay was selected from the final unplug/reconnect tests. Safety still occurs at 200 ms because the bridge neutralizes the controller state before waiting for device destruction.

## Install

```bash
cd rdk_controller_bridge_final_v2
sudo ./install.sh
sudo ./enable.sh
```

The installer handles the dependency problem encountered with the earlier version. It will:

1. install BlueZ and build dependencies;
2. try a distro `interception-tools` package if available;
3. if a valid Interception Tools `uinput` is still unavailable, build Interception Tools v0.6.8 from source;
4. install the verified `uinput` at `/usr/local/bin/uinput`;
5. load the kernel `uinput` module and verify `/dev/uinput` exists;
6. install the working DualSense YAML unless `/etc/controller/dualsense.yaml` already exists;
7. install and enable only the transport/bridge components.

If an older revision of this package installed `remote-joy*` helper services, this installer disables and removes those helpers. Your separately managed ROS launch files are not modified.

## Services

Only these services are installed by this package:

- `rdk-rfcomm.service`
- `controller-bridge.service`

The RFCOMM listener uses the command verified during final testing:

```text
rfcomm -r -i hci0 watch 0 1
```

After the bridge creates `Remote DualSense`, use your existing ROS configuration exactly as you normally would. ROS is intentionally outside this package.

## Healthy logs

Bluetooth-only, with Wi-Fi disabled:

```text
state=NORMAL
udp=<not increasing>
bt=<increasing>
age=0.000..0.05s
pending=0
malformed=0
```

With both paths active, `udp` and `bt` both increase. `dup/late` also increases because the second copy of each redundant packet is intentionally discarded.

## Diagnostics

```bash
journalctl -u controller-bridge.service -f
journalctl -u rdk-rfcomm.service -f
sudo ./diagnose.sh
```

## Important

The bridge provides a controller-input fail-safe. Motor/actuator command nodes should still have their own independent command timeout/watchdog.
