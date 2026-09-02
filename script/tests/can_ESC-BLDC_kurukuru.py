#!/usr/bin/env python3
"""
VESC CAN connector wiggle test.

This sends a small repeated motor-current pattern so CAN dropouts become
visible as a motor stop or missed twitch while you gently wiggle connectors.

Default pattern:
  +0.5 A for 0.4 s, then -0.5 A for 0.4 s, repeated

Use a low current first. Keep the motor unloaded and mechanically safe.

Example:
  python3 vesc_can_wiggle_test.py --controller-id 0x32 --current 0.5
  python3 vesc_can_wiggle_test.py --controller-id 0x32 --current 0.8 --duration 30 --yes
  python3 vesc_can_wiggle_test.py --controller-id 0x32 --mode step --current 0.5 --low-current 0.2
"""

import argparse
import struct
import time
from typing import Optional

import can


PACKET_SET_CURRENT = 1
PACKET_SET_RPM = 3
PACKET_STATUS = 9
PACKET_STATUS_4 = 16


def parse_int(value: str) -> int:
    return int(value, 0)


def vesc_id(packet_id: int, controller_id: int) -> int:
    return ((packet_id & 0xFF) << 8) | (controller_id & 0xFF)


def hex_data(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def be_i16(data: bytes, offset: int) -> int:
    return struct.unpack(">h", data[offset : offset + 2])[0]


def be_i32(data: bytes, offset: int) -> int:
    return struct.unpack(">i", data[offset : offset + 4])[0]


def open_bus(args: argparse.Namespace) -> can.BusABC:
    return can.interface.Bus(
        channel=args.channel,
        interface="socketcan",
        bitrate=args.bitrate,
    )


def send_current(bus: Optional[can.BusABC], controller_id: int, current_a: float, dry_run: bool) -> None:
    raw = int(current_a * 1000.0)
    data = raw.to_bytes(4, "big", signed=True)
    arb_id = vesc_id(PACKET_SET_CURRENT, controller_id)
    if dry_run:
        print(f"tx current={current_a:+.3f}A EXT id=0x{arb_id:X} data={hex_data(data)}")
        return
    if bus is None:
        raise RuntimeError("bus is not open")
    bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))


def send_zero(bus: Optional[can.BusABC], controller_id: int, dry_run: bool) -> None:
    send_current(bus, controller_id, 0.0, dry_run)
    data = (0).to_bytes(4, "big", signed=True)
    arb_id = vesc_id(PACKET_SET_RPM, controller_id)
    if dry_run:
        print(f"tx rpm=0 EXT id=0x{arb_id:X} data={hex_data(data)}")
        return
    if bus is None:
        raise RuntimeError("bus is not open")
    bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))


def decode_status(msg: can.Message, controller_id: int) -> Optional[str]:
    packet_id = (msg.arbitration_id >> 8) & 0xFF
    msg_controller_id = msg.arbitration_id & 0xFF
    data = bytes(msg.data)

    if not msg.is_extended_id or msg_controller_id != controller_id or len(data) < 8:
        return None

    if packet_id == PACKET_STATUS:
        rpm = be_i32(data, 0)
        current = be_i16(data, 4) / 10.0
        duty = be_i16(data, 6) / 1000.0
        return f"rpm={rpm:+7d} current={current:+5.1f}A duty={duty:+.3f}"

    if packet_id == PACKET_STATUS_4:
        temp_fet = be_i16(data, 0) / 10.0
        current_in = be_i16(data, 4) / 10.0
        return f"fet={temp_fet:4.1f}C input_current={current_in:+5.1f}A"

    return None


def pattern_current(args: argparse.Namespace, elapsed: float) -> tuple[float, str]:
    if args.mode == "constant":
        return args.current, "constant"

    phase = int(elapsed / args.half_period) % 2

    if args.mode == "step":
        if phase == 0:
            return args.current, "high"
        return args.low_current, "low"

    if args.mode == "pulse":
        if phase == 0:
            return args.current, "on"
        return 0.0, "off"

    if phase == 0:
        return args.current, "forward"
    return -args.current, "reverse"


def require_confirm(args: argparse.Namespace) -> None:
    if args.yes or args.dry_run:
        return
    print(
        f"Controller 0x{args.controller_id:02X}: {args.mode} pattern, "
        f"current={args.current}A, half_period={args.half_period}s, "
        f"duration={'forever' if args.duration <= 0 else str(args.duration) + 's'}."
    )
    print("Keep the motor unloaded and clear of fingers/tools.")
    input("Press Enter to start, or Ctrl+C to cancel. ")


def main() -> None:
    parser = argparse.ArgumentParser(description="Make VESC CAN dropouts visible with a low-current wiggle.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=500000)
    parser.add_argument("--controller-id", type=parse_int, default=0x32)
    parser.add_argument("--current", type=float, default=0.5, help="Motor current in A, default: 0.5")
    parser.add_argument("--half-period", type=float, default=0.4, help="Seconds per phase, default: 0.4")
    parser.add_argument("--command-period", type=float, default=0.05, help="CAN command period, default: 0.05")
    parser.add_argument("--duration", type=float, default=0.0, help="Total seconds; 0 means run until Ctrl+C")
    parser.add_argument("--rx-timeout", type=float, default=0.6, help="Seconds without status before RX LOST")
    parser.add_argument("--mode", choices=("alternate", "pulse", "constant", "step"), default="alternate")
    parser.add_argument("--low-current", type=float, default=0.2, help="Low nonzero current for step mode, default: 0.2")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.current < 0:
        raise SystemExit("--current must be positive; direction is controlled by --mode")
    if args.low_current < 0:
        raise SystemExit("--low-current must be zero or positive")
    if args.half_period <= 0:
        raise SystemExit("--half-period must be positive")
    if args.command_period <= 0:
        raise SystemExit("--command-period must be positive")

    require_confirm(args)

    bus = None if args.dry_run else open_bus(args)
    start = time.monotonic()
    next_cmd = start
    next_print = start
    last_rx = time.monotonic()
    last_status = "no status yet"
    last_phase = None
    rx_lost_printed = False

    try:
        print("Running. Wiggle the connector gently; Ctrl+C stops and sends zero.")
        while True:
            now = time.monotonic()
            elapsed = now - start
            if args.duration > 0 and elapsed >= args.duration:
                break

            current, phase = pattern_current(args, elapsed)

            if phase != last_phase:
                print(f"phase={phase:<8} command_current={current:+.3f}A")
                last_phase = phase

            if now >= next_cmd:
                send_current(bus, args.controller_id, current, args.dry_run)
                next_cmd = now + args.command_period

            if bus is not None:
                msg = bus.recv(timeout=0.01)
                if msg is not None:
                    decoded = decode_status(msg, args.controller_id)
                    if decoded is not None:
                        last_rx = now
                        last_status = decoded
                        if rx_lost_printed:
                            print("RX OK again")
                            rx_lost_printed = False
            else:
                time.sleep(min(args.command_period, 0.02))

            if now - last_rx > args.rx_timeout and not rx_lost_printed:
                print(f"RX LOST: no VESC status for {now - last_rx:.2f}s")
                rx_lost_printed = True

            if now >= next_print:
                print(f"status: {last_status}")
                next_print = now + 0.5

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            print("Sending zero...")
            for _ in range(5):
                send_zero(bus, args.controller_id, args.dry_run)
                time.sleep(0.03)
        finally:
            if bus is not None:
                bus.shutdown()


if __name__ == "__main__":
    main()

