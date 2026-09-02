#!/usr/bin/env python3
"""
Small VESC CAN jog tool for the 1 Mbps mixed bus.

Default target:
  ESC controller ID 0x34 on can0 at 1 Mbps.

Examples:
  python3 vesc_esc_jog_1mbps.py --mode status --seconds 3
  python3 vesc_esc_jog_1mbps.py --mode ping
  python3 vesc_esc_jog_1mbps.py --mode zero
  python3 vesc_esc_jog_1mbps.py --mode current --current 0.3 --duration 0.5
  python3 vesc_esc_jog_1mbps.py --mode duty --duty 0.01 --duration 0.5
  python3 vesc_esc_jog_1mbps.py --mode alternate --current 0.5 --half-period 0.4 --duration 5 --yes
"""

from __future__ import annotations

import argparse
import struct
import time


PACKET_SET_DUTY = 0
PACKET_SET_CURRENT = 1
PACKET_SET_CURRENT_BRAKE = 2
PACKET_SET_RPM = 3
PACKET_STATUS = 9
PACKET_STATUS_2 = 14
PACKET_STATUS_3 = 15
PACKET_STATUS_4 = 16
PACKET_PING = 17
PACKET_PONG = 18
PACKET_STATUS_5 = 27

PACKET_NAMES = {
    PACKET_SET_DUTY: "SET_DUTY",
    PACKET_SET_CURRENT: "SET_CURRENT",
    PACKET_SET_CURRENT_BRAKE: "SET_CURRENT_BRAKE",
    PACKET_SET_RPM: "SET_RPM",
    PACKET_STATUS: "STATUS",
    PACKET_STATUS_2: "STATUS_2",
    PACKET_STATUS_3: "STATUS_3",
    PACKET_STATUS_4: "STATUS_4",
    PACKET_PING: "PING",
    PACKET_PONG: "PONG",
    PACKET_STATUS_5: "STATUS_5",
}

can = None


def load_can_module():
    global can
    if can is None:
        try:
            import can as can_module
        except ModuleNotFoundError as exc:
            raise SystemExit("python-can is required on the RDK: pip3 install python-can") from exc
        can = can_module
    return can


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


def open_bus(args):
    can_module = load_can_module()
    return can_module.interface.Bus(channel=args.channel, interface="socketcan", bitrate=args.bitrate)


def send_ext(bus, packet_id: int, controller_id: int, data: bytes, label: str, dry_run: bool) -> None:
    can_module = load_can_module()
    arb_id = vesc_id(packet_id, controller_id)
    print(f"tx {label:<14} EXT id=0x{arb_id:X} data={hex_data(data)}")
    if dry_run:
        return
    bus.send(can_module.Message(arbitration_id=arb_id, data=data, is_extended_id=True), timeout=0.05)


def set_duty(bus, args, duty: float) -> None:
    raw = int(duty * 100000.0)
    send_ext(bus, PACKET_SET_DUTY, args.controller_id, raw.to_bytes(4, "big", signed=True), f"duty={duty:+.4f}", args.dry_run)


def set_current(bus, args, current_a: float) -> None:
    raw = int(current_a * 1000.0)
    send_ext(bus, PACKET_SET_CURRENT, args.controller_id, raw.to_bytes(4, "big", signed=True), f"current={current_a:+.3f}A", args.dry_run)


def set_brake_current(bus, args, current_a: float) -> None:
    raw = int(current_a * 1000.0)
    send_ext(bus, PACKET_SET_CURRENT_BRAKE, args.controller_id, raw.to_bytes(4, "big", signed=True), f"brake={current_a:+.3f}A", args.dry_run)


def set_rpm(bus, args, rpm: int) -> None:
    send_ext(bus, PACKET_SET_RPM, args.controller_id, int(rpm).to_bytes(4, "big", signed=True), f"rpm={rpm:+d}", args.dry_run)


def send_zero(bus, args) -> None:
    set_duty(bus, args, 0.0)
    time.sleep(args.gap)
    set_current(bus, args, 0.0)
    time.sleep(args.gap)
    set_rpm(bus, args, 0)


def decode_status(msg) -> str:
    packet = (msg.arbitration_id >> 8) & 0xFF
    controller_id = msg.arbitration_id & 0xFF
    data = bytes(msg.data)
    name = PACKET_NAMES.get(packet, f"PACKET_{packet}")

    if packet == PACKET_STATUS and len(data) >= 8:
        rpm = be_i32(data, 0)
        current = be_i16(data, 4) / 10.0
        duty = be_i16(data, 6) / 1000.0
        return f"id=0x{controller_id:02X} {name} rpm={rpm:+d} current={current:+.1f}A duty={duty:+.3f}"

    if packet == PACKET_STATUS_4 and len(data) >= 8:
        temp_fet = be_i16(data, 0) / 10.0
        temp_motor = be_i16(data, 2) / 10.0
        current_in = be_i16(data, 4) / 10.0
        return f"id=0x{controller_id:02X} {name} fet={temp_fet:.1f}C motor={temp_motor:.1f}C input_current={current_in:+.1f}A"

    if packet == PACKET_STATUS_5 and len(data) >= 6:
        tacho = be_i32(data, 0)
        vin = be_i16(data, 4) / 10.0
        return f"id=0x{controller_id:02X} {name} tacho={tacho} vin={vin:.1f}V"

    if packet == PACKET_PONG:
        payload = hex_data(data)
        pong_from = f"0x{data[0]:02X}" if data else "?"
        return f"id=0x{controller_id:02X} {name} pong_from={pong_from} payload={payload}"

    return f"id=0x{controller_id:02X} {name} data={hex_data(data)}"


def recv_status(bus, seconds: float, target_id: int | None = None) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is None or not msg.is_extended_id:
            continue
        packet = (msg.arbitration_id >> 8) & 0xFF
        controller_id = msg.arbitration_id & 0xFF
        if target_id is not None and controller_id not in (target_id, 0xFD):
            continue
        if packet not in PACKET_NAMES:
            continue
        print(f"rx {decode_status(msg)}")


def confirm_if_needed(args, description: str) -> None:
    if args.yes or args.dry_run:
        return
    print(description)
    print("Make sure the wheel is unloaded and the emergency stop/power switch is reachable.")
    input("Press Enter to send, or Ctrl+C to cancel. ")


def safety_check(args) -> None:
    if args.allow_high:
        return
    if abs(args.current) > 3.0:
        raise SystemExit("Refusing current > 3A without --allow-high.")
    if abs(args.duty) > 0.08:
        raise SystemExit("Refusing duty > 0.08 without --allow-high.")
    if abs(args.rpm) > 3000:
        raise SystemExit("Refusing rpm > 3000 ERPM without --allow-high.")


def run_command_loop(bus, args, command_func) -> None:
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            command_func()
            recv_status(bus, min(args.period, 0.08), target_id=args.controller_id)
    except KeyboardInterrupt:
        print("\nInterrupted; sending zero.")
    finally:
        send_zero(bus, args)


def main() -> None:
    parser = argparse.ArgumentParser(description="Jog one VESC-compatible ESC on a 1 Mbps CAN bus.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--controller-id", type=parse_int, default=0x34)
    parser.add_argument("--host-id", type=parse_int, default=0xFD)
    parser.add_argument("--mode", choices=("status", "ping", "zero", "current", "duty", "rpm", "alternate", "brake"), default="status")
    parser.add_argument("--seconds", type=float, default=5.0, help="status listen seconds")
    parser.add_argument("--duration", type=float, default=0.5, help="moving command duration")
    parser.add_argument("--period", type=float, default=0.05, help="command period")
    parser.add_argument("--gap", type=float, default=0.03, help="gap between zero frames")
    parser.add_argument("--current", type=float, default=0.3, help="motor current in A")
    parser.add_argument("--duty", type=float, default=0.01, help="duty ratio")
    parser.add_argument("--rpm", type=int, default=300, help="electrical RPM")
    parser.add_argument("--half-period", type=float, default=0.4, help="alternate direction phase time")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--allow-high", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.period <= 0:
        raise SystemExit("--period must be positive")
    if args.half_period <= 0:
        raise SystemExit("--half-period must be positive")
    safety_check(args)

    bus = None if args.dry_run else open_bus(args)
    try:
        if args.mode == "status":
            print(f"Listening for VESC status on {args.channel} at {args.bitrate} bps...")
            recv_status(bus, args.seconds, target_id=args.controller_id)
            return

        if args.mode == "ping":
            send_ext(bus, PACKET_PING, args.controller_id, bytes([args.host_id & 0xFF]), "ping", args.dry_run)
            if bus is not None:
                recv_status(bus, args.seconds, target_id=args.controller_id)
            return

        if args.mode == "zero":
            send_zero(bus, args)
            if bus is not None:
                recv_status(bus, min(args.seconds, 1.0), target_id=args.controller_id)
            return

        confirm_if_needed(
            args,
            f"About to command ESC 0x{args.controller_id:02X} in {args.mode} mode for {args.duration}s.",
        )

        if args.mode == "current":
            run_command_loop(bus, args, lambda: set_current(bus, args, args.current))
        elif args.mode == "duty":
            run_command_loop(bus, args, lambda: set_duty(bus, args, args.duty))
        elif args.mode == "rpm":
            run_command_loop(bus, args, lambda: set_rpm(bus, args, args.rpm))
        elif args.mode == "brake":
            run_command_loop(bus, args, lambda: set_brake_current(bus, args, abs(args.current)))
        elif args.mode == "alternate":
            start = time.monotonic()

            def alternate_command() -> None:
                elapsed = time.monotonic() - start
                sign = 1.0 if int(elapsed / args.half_period) % 2 == 0 else -1.0
                set_current(bus, args, sign * abs(args.current))

            run_command_loop(bus, args, alternate_command)
    finally:
        if bus is not None:
            bus.shutdown()


if __name__ == "__main__":
    main()

