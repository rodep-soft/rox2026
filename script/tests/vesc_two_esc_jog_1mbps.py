#!/usr/bin/env python3
"""
Jog two VESC-compatible ESCs on the 1 Mbps mixed CAN bus.

Defaults:
  ESC IDs: 0x33, 0x34
  CAN: can0 at 1 Mbps

VESC CAN SET_RPM uses ERPM in most VESC firmware builds:
  ERPM = mechanical_rpm * pole_pairs

Examples:
  python3 vesc_two_esc_jog_1mbps.py --mode status --seconds 3
  python3 vesc_two_esc_jog_1mbps.py --mode ping
  python3 vesc_two_esc_jog_1mbps.py --mode zero
  python3 vesc_two_esc_jog_1mbps.py --mode rpm --rpm 500 --duration 1 --yes
  python3 vesc_two_esc_jog_1mbps.py --mode rpm --mechanical-rpm 100 --pole-pairs 7 --duration 1 --yes
  python3 vesc_two_esc_jog_1mbps.py --mode alternate-rpm --rpm 800 --half-period 0.5 --duration 10 --yes
  python3 vesc_two_esc_jog_1mbps.py --mode current --current 10 --duration 1 --allow-high --yes
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


def parse_ids(value: str) -> list[int]:
    ids = [parse_int(part.strip()) for part in value.split(",") if part.strip()]
    if not ids:
        raise argparse.ArgumentTypeError("at least one ESC ID is required")
    for controller_id in ids:
        if not 0 <= controller_id <= 0xFF:
            raise argparse.ArgumentTypeError(f"invalid ESC ID: 0x{controller_id:X}")
    return ids


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


def sign_for_id(args, controller_id: int) -> int:
    return -1 if controller_id in args.reverse_ids else 1


def send_ext(bus, packet_id: int, controller_id: int, data: bytes, label: str, dry_run: bool) -> None:
    can_module = load_can_module()
    arb_id = vesc_id(packet_id, controller_id)
    print(f"tx id=0x{controller_id:02X} {label:<16} arb=0x{arb_id:X} data={hex_data(data)}")
    if dry_run:
        return
    bus.send(can_module.Message(arbitration_id=arb_id, data=data, is_extended_id=True), timeout=0.05)


def set_current(bus, args, controller_id: int, current_a: float) -> None:
    raw = int(current_a * 1000.0)
    send_ext(
        bus,
        PACKET_SET_CURRENT,
        controller_id,
        raw.to_bytes(4, "big", signed=True),
        f"current={current_a:+.3f}A",
        args.dry_run,
    )


def set_brake_current(bus, args, controller_id: int, current_a: float) -> None:
    raw = int(abs(current_a) * 1000.0)
    send_ext(
        bus,
        PACKET_SET_CURRENT_BRAKE,
        controller_id,
        raw.to_bytes(4, "big", signed=True),
        f"brake={abs(current_a):+.3f}A",
        args.dry_run,
    )


def set_rpm(bus, args, controller_id: int, rpm: int) -> None:
    send_ext(
        bus,
        PACKET_SET_RPM,
        controller_id,
        int(rpm).to_bytes(4, "big", signed=True),
        f"rpm={rpm:+d}",
        args.dry_run,
    )


def send_zero(bus, args) -> None:
    for controller_id in args.ids:
        set_current(bus, args, controller_id, 0.0)
        time.sleep(args.inter_frame_gap)
    for controller_id in args.ids:
        set_rpm(bus, args, controller_id, 0)
        time.sleep(args.inter_frame_gap)


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


def should_print(args, msg) -> bool:
    if not msg.is_extended_id:
        return False
    packet = (msg.arbitration_id >> 8) & 0xFF
    controller_id = msg.arbitration_id & 0xFF
    data = bytes(msg.data)

    if packet == PACKET_PONG and controller_id == args.host_id:
        return bool(data) and data[0] in args.ids
    return controller_id in args.ids and packet in PACKET_NAMES


def recv_status(bus, args, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.03)
        if msg is None:
            continue
        if should_print(args, msg):
            print(f"rx {decode_status(msg)}")


def selected_rpm(args) -> int:
    if args.mechanical_rpm is None:
        return int(args.rpm)
    return int(round(args.mechanical_rpm * args.pole_pairs))


def safety_check(args) -> None:
    rpm = selected_rpm(args)
    if args.allow_high:
        return
    if abs(args.current) > 3.0:
        raise SystemExit("Refusing current > 3A without --allow-high.")
    if abs(rpm) > 3000:
        raise SystemExit("Refusing rpm > 3000 ERPM without --allow-high.")


def confirm_if_needed(args, description: str) -> None:
    if args.yes or args.dry_run:
        return
    print(description)
    print("Make sure both wheels are unloaded and power can be cut immediately.")
    input("Press Enter to send, or Ctrl+C to cancel. ")


def send_all_current(bus, args, current_a: float) -> None:
    for controller_id in args.ids:
        set_current(bus, args, controller_id, sign_for_id(args, controller_id) * current_a)
        time.sleep(args.inter_frame_gap)


def send_all_rpm(bus, args, rpm: int) -> None:
    for controller_id in args.ids:
        set_rpm(bus, args, controller_id, sign_for_id(args, controller_id) * rpm)
        time.sleep(args.inter_frame_gap)


def send_all_brake(bus, args, current_a: float) -> None:
    for controller_id in args.ids:
        set_brake_current(bus, args, controller_id, current_a)
        time.sleep(args.inter_frame_gap)


def run_loop(bus, args, command_func) -> None:
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            command_func()
            recv_status(bus, args, min(args.period, 0.08))
    except KeyboardInterrupt:
        print("\nInterrupted; sending zero.")
    finally:
        send_zero(bus, args)


def main() -> None:
    parser = argparse.ArgumentParser(description="Jog two VESC-compatible ESCs on a 1 Mbps CAN bus.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--ids", type=parse_ids, default=parse_ids("0x33,0x34"))
    parser.add_argument("--reverse-ids", type=parse_ids, default=[], help="comma-separated IDs whose sign should be inverted")
    parser.add_argument("--host-id", type=parse_int, default=0xFD)
    parser.add_argument(
        "--mode",
        choices=("status", "ping", "zero", "current", "rpm", "alternate-current", "alternate-rpm", "brake"),
        default="status",
    )
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--inter-frame-gap", type=float, default=0.01)
    parser.add_argument("--current", type=float, default=0.5, help="motor current in A")
    parser.add_argument("--rpm", type=int, default=500, help="VESC ERPM command")
    parser.add_argument("--mechanical-rpm", type=float, default=None, help="mechanical RPM; converted to ERPM using --pole-pairs")
    parser.add_argument("--pole-pairs", type=int, default=7)
    parser.add_argument("--half-period", type=float, default=0.5)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--allow-high", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    args.reverse_ids = set(args.reverse_ids)
    unknown_reverse = args.reverse_ids.difference(args.ids)
    if unknown_reverse:
        raise SystemExit("--reverse-ids must be a subset of --ids")
    if args.period <= 0 or args.inter_frame_gap < 0 or args.half_period <= 0:
        raise SystemExit("periods and gaps must be positive")
    safety_check(args)

    bus = None if args.dry_run else open_bus(args)
    try:
        if args.mode == "status":
            print(f"Listening for VESC status from {', '.join(f'0x{x:02X}' for x in args.ids)}...")
            recv_status(bus, args, args.seconds)
            return

        if args.mode == "ping":
            for controller_id in args.ids:
                send_ext(bus, PACKET_PING, controller_id, bytes([args.host_id & 0xFF]), "ping", args.dry_run)
                time.sleep(args.inter_frame_gap)
            if bus is not None:
                recv_status(bus, args, args.seconds)
            return

        if args.mode == "zero":
            send_zero(bus, args)
            if bus is not None:
                recv_status(bus, args, min(args.seconds, 1.0))
            return

        target = ", ".join(f"0x{x:02X}" for x in args.ids)
        confirm_if_needed(args, f"About to command ESCs {target} in {args.mode} mode for {args.duration}s.")

        if args.mode == "current":
            run_loop(bus, args, lambda: send_all_current(bus, args, args.current))
        elif args.mode == "rpm":
            rpm = selected_rpm(args)
            print(f"Using VESC SET_RPM target {rpm:+d} ERPM")
            run_loop(bus, args, lambda: send_all_rpm(bus, args, rpm))
        elif args.mode == "alternate-current":
            start = time.monotonic()

            def alternate_current() -> None:
                sign = 1.0 if int((time.monotonic() - start) / args.half_period) % 2 == 0 else -1.0
                send_all_current(bus, args, sign * abs(args.current))

            run_loop(bus, args, alternate_current)
        elif args.mode == "alternate-rpm":
            rpm = abs(selected_rpm(args))
            print(f"Using VESC SET_RPM target +/-{rpm} ERPM")
            start = time.monotonic()

            def alternate_rpm() -> None:
                sign = 1 if int((time.monotonic() - start) / args.half_period) % 2 == 0 else -1
                send_all_rpm(bus, args, sign * rpm)

            run_loop(bus, args, alternate_rpm)
        elif args.mode == "brake":
            run_loop(bus, args, lambda: send_all_brake(bus, args, args.current))
    finally:
        if bus is not None:
            bus.shutdown()


if __name__ == "__main__":
    main()

