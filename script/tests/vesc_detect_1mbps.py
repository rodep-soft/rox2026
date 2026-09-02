#!/usr/bin/env python3
"""
Detect VESC-compatible ESC controller IDs on a 1 Mbps CAN bus.

This script is non-motion:
  - passively listens for VESC status frames
  - actively sends VESC CAN PING frames and listens for PONG replies
  - also prints RobStride-looking frames so mixed buses are easier to read

VESC extended CAN ID format:
  arbitration_id = (packet_id << 8) | controller_id

Useful packet IDs:
  9  STATUS
  14 STATUS_2
  15 STATUS_3
  16 STATUS_4
  17 PING
  18 PONG

Example:
  python3 vesc_detect_1mbps.py
  python3 vesc_detect_1mbps.py --ping-scan --start 0x01 --end 0x7F
"""

import argparse
import struct
import time

import can


PACKET_STATUS = 9
PACKET_STATUS_2 = 14
PACKET_STATUS_3 = 15
PACKET_STATUS_4 = 16
PACKET_PING = 17
PACKET_PONG = 18
PACKET_STATUS_5 = 27

VESC_PACKET_NAMES = {
    PACKET_STATUS: "STATUS",
    PACKET_STATUS_2: "STATUS_2",
    PACKET_STATUS_3: "STATUS_3",
    PACKET_STATUS_4: "STATUS_4",
    PACKET_PING: "PING",
    PACKET_PONG: "PONG",
    PACKET_STATUS_5: "STATUS_5",
}

ROBSTRIDE_COMM_NAMES = {
    0x00: "GET_ID",
    0x01: "CONTROL",
    0x02: "FEEDBACK",
    0x03: "ENABLE",
    0x04: "STOP",
    0x11: "READ",
    0x12: "WRITE",
    0x18: "ACTIVE_REPORT",
}


def parse_int(value: str) -> int:
    return int(value, 0)


def be_i16(data: bytes, offset: int) -> int:
    return struct.unpack(">h", data[offset : offset + 2])[0]


def be_i32(data: bytes, offset: int) -> int:
    return struct.unpack(">i", data[offset : offset + 4])[0]


def hex_data(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def vesc_arb_id(packet_id: int, controller_id: int) -> int:
    return ((packet_id & 0xFF) << 8) | (controller_id & 0xFF)


def robstride_parts(arb_id: int) -> tuple[int, int, int]:
    comm_type = (arb_id >> 24) & 0x1F
    data2 = (arb_id >> 8) & 0xFFFF
    dest = arb_id & 0xFF
    return comm_type, data2, dest


def decode_vesc(msg: can.Message) -> tuple[int, str] | None:
    if not msg.is_extended_id:
        return None

    packet_id = (msg.arbitration_id >> 8) & 0xFF
    controller_id = msg.arbitration_id & 0xFF
    name = VESC_PACKET_NAMES.get(packet_id)
    if name is None:
        return None

    data = bytes(msg.data)
    summary = f"VESC id=0x{controller_id:02X} packet={packet_id}({name}) arb=0x{msg.arbitration_id:X}"

    if packet_id == PACKET_STATUS and len(data) >= 8:
        rpm = be_i32(data, 0)
        current = be_i16(data, 4) / 10.0
        duty = be_i16(data, 6) / 1000.0
        summary += f" rpm={rpm} current={current:.1f}A duty={duty:.3f}"

    elif packet_id == PACKET_STATUS_4 and len(data) >= 8:
        temp_fet = be_i16(data, 0) / 10.0
        temp_motor = be_i16(data, 2) / 10.0
        current_in = be_i16(data, 4) / 10.0
        summary += f" fet={temp_fet:.1f}C motor={temp_motor:.1f}C input={current_in:.1f}A"

    elif packet_id == PACKET_PONG:
        payload = hex_data(data)
        if len(data) >= 1:
            summary += f" pong_from=0x{data[0]:02X} payload={payload}"
        else:
            summary += " pong_empty"

    return controller_id, summary


def decode_robstride(msg: can.Message) -> tuple[int, str] | None:
    if not msg.is_extended_id:
        return None

    comm_type, data2, dest = robstride_parts(msg.arbitration_id)
    name = ROBSTRIDE_COMM_NAMES.get(comm_type)
    if name is None:
        return None

    # VESC IDs such as 0x932 also look like comm_type 0 if decoded blindly.
    # Treat only RobStride's characteristic reply/control IDs as RobStride.
    if comm_type == 0x00 and dest != 0xFE:
        return None

    motor_id = data2 & 0xFF
    summary = (
        f"RobStride? motor=0x{motor_id:02X} comm=0x{comm_type:X}({name}) "
        f"dest=0x{dest:02X} arb=0x{msg.arbitration_id:X} data={hex_data(bytes(msg.data))}"
    )
    return motor_id, summary


def open_bus(args: argparse.Namespace) -> can.BusABC:
    return can.interface.Bus(
        channel=args.channel,
        interface="socketcan",
        bitrate=args.bitrate,
    )


def listen(bus: can.BusABC, seconds: float, verbose_raw: bool) -> tuple[set[int], set[int]]:
    vesc_ids = set()
    robstride_ids = set()
    deadline = time.monotonic() + seconds

    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue

        vesc = decode_vesc(msg)
        if vesc is not None:
            controller_id, summary = vesc
            vesc_ids.add(controller_id)
            print("rx " + summary)
            continue

        robstride = decode_robstride(msg)
        if robstride is not None:
            motor_id, summary = robstride
            robstride_ids.add(motor_id)
            print("rx " + summary)
            continue

        if verbose_raw:
            kind = "EXT" if msg.is_extended_id else "STD"
            print(f"rx raw {kind} arb=0x{msg.arbitration_id:X} data={hex_data(bytes(msg.data))}")

    return vesc_ids, robstride_ids


def ping_scan(bus: can.BusABC, args: argparse.Namespace) -> set[int]:
    found = set()
    pong_arb_id = vesc_arb_id(PACKET_PONG, args.host_id)

    print(
        f"Sending VESC PING scan 0x{args.start:02X}..0x{args.end:02X}; "
        f"expect PONG arb=0x{pong_arb_id:X}"
    )

    for controller_id in range(args.start, args.end + 1):
        tx_id = vesc_arb_id(PACKET_PING, controller_id)
        data = bytes([args.host_id & 0xFF])
        if args.verbose_ping:
            print(f"tx ping target=0x{controller_id:02X} arb=0x{tx_id:X} data={hex_data(data)}")
        try:
            bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True), timeout=args.send_timeout)
        except can.CanOperationError as exc:
            print(f"tx ping target=0x{controller_id:02X} failed: {exc}")
            continue

        deadline = time.monotonic() + args.pong_timeout
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.002)
            if msg is None or not msg.is_extended_id:
                continue

            vesc = decode_vesc(msg)
            if vesc is not None:
                reply_dest, summary = vesc
                packet_id = (msg.arbitration_id >> 8) & 0xFF
                if packet_id == PACKET_PONG and reply_dest == args.host_id:
                    if len(msg.data) >= 1:
                        found.add(msg.data[0])
                    print("rx " + summary)

        time.sleep(args.ping_gap)

    return found


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect VESC ESC IDs on a 1 Mbps mixed CAN bus.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--seconds", type=float, default=5.0, help="passive listen seconds")
    parser.add_argument("--ping-scan", action="store_true", help="actively ping possible VESC controller IDs")
    parser.add_argument("--start", type=parse_int, default=0x01)
    parser.add_argument("--end", type=parse_int, default=0x7F)
    parser.add_argument("--host-id", type=parse_int, default=0xFD)
    parser.add_argument("--pong-timeout", type=float, default=0.02)
    parser.add_argument("--ping-gap", type=float, default=0.005)
    parser.add_argument("--send-timeout", type=float, default=0.01)
    parser.add_argument("--verbose-raw", action="store_true")
    parser.add_argument("--verbose-ping", action="store_true")
    args = parser.parse_args()

    bus = open_bus(args)
    all_vesc_ids = set()
    all_robstride_ids = set()

    try:
        if args.seconds > 0:
            print(f"Passive listen on {args.channel} at {args.bitrate} bps for {args.seconds}s...")
            vesc_ids, robstride_ids = listen(bus, args.seconds, args.verbose_raw)
            all_vesc_ids.update(vesc_ids)
            all_robstride_ids.update(robstride_ids)

        if args.ping_scan:
            found_by_ping = ping_scan(bus, args)
            all_vesc_ids.update(found_by_ping)

        if all_vesc_ids:
            print("Found VESC ESC controller IDs: " + ", ".join(f"0x{x:02X}" for x in sorted(all_vesc_ids)))
        else:
            print("No VESC ESC controller IDs found.")

        if all_robstride_ids:
            print("Also saw RobStride-like motor IDs: " + ", ".join(f"0x{x:02X}" for x in sorted(all_robstride_ids)))

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()

