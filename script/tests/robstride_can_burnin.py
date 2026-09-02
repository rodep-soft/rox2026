#!/usr/bin/env python3
"""
RobStride CAN non-motion burn-in test.

This repeatedly sends get-id frames to one RobStride motor and checks whether
the Linux CAN error counters recover toward zero or climb again.

Example:
  python3 robstride_can_burnin.py --motor-id 0x23 --count 200
"""

import argparse
import re
import subprocess
import time

import can


COMM_GET_ID = 0x00


def parse_int(value: str) -> int:
    return int(value, 0)


def make_ext_id(comm_type: int, target_id: int, data2: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def open_bus(args: argparse.Namespace) -> can.BusABC:
    return can.interface.Bus(
        channel=args.channel,
        interface="socketcan",
        bitrate=args.bitrate,
    )


def ip_stats(channel: str) -> str:
    result = subprocess.run(
        ["ip", "-details", "-statistics", "link", "show", channel],
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout


def summarize_ip_stats(text: str) -> str:
    state_match = re.search(r"can state ([A-Z-]+) \(berr-counter tx (\d+) rx (\d+)\)", text)
    totals_match = re.search(
        r"re-started bus-errors arbit-lost error-warn error-pass bus-off\s+"
        r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
        text,
        re.MULTILINE,
    )

    parts = []
    if state_match:
        parts.append(
            f"state={state_match.group(1)} txerr={state_match.group(2)} rxerr={state_match.group(3)}"
        )
    if totals_match:
        parts.append(
            "totals="
            f"restart:{totals_match.group(1)} "
            f"warn:{totals_match.group(4)} "
            f"pass:{totals_match.group(5)} "
            f"busoff:{totals_match.group(6)}"
        )
    return " ".join(parts) if parts else "could not parse ip stats"


def get_id_once(bus: can.BusABC, args: argparse.Namespace) -> bool:
    tx_id = make_ext_id(COMM_GET_ID, args.motor_id, args.host_id)
    expected_rx_id = ((args.motor_id & 0xFFFF) << 8) | 0xFE
    bus.send(can.Message(arbitration_id=tx_id, data=bytes(8), is_extended_id=True), timeout=args.send_timeout)

    deadline = time.monotonic() + args.reply_timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.002)
        if msg is None or not msg.is_extended_id:
            continue
        if msg.arbitration_id == expected_rx_id:
            return True
    return False


def main() -> None:
    parser = argparse.ArgumentParser(description="Non-motion RobStride CAN error-counter burn-in.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--motor-id", type=parse_int, required=True)
    parser.add_argument("--host-id", type=parse_int, default=0xFD)
    parser.add_argument("--count", type=int, default=200)
    parser.add_argument("--interval", type=float, default=0.02)
    parser.add_argument("--reply-timeout", type=float, default=0.03)
    parser.add_argument("--send-timeout", type=float, default=0.02)
    parser.add_argument("--print-every", type=int, default=10)
    args = parser.parse_args()

    bus = open_bus(args)
    ok_count = 0
    miss_count = 0

    try:
        print("start " + summarize_ip_stats(ip_stats(args.channel)))
        for index in range(1, args.count + 1):
            try:
                ok = get_id_once(bus, args)
            except can.CanOperationError as exc:
                ok = False
                print(f"{index:04d} TX ERROR: {exc}")

            if ok:
                ok_count += 1
            else:
                miss_count += 1

            if index % args.print_every == 0 or not ok:
                status = summarize_ip_stats(ip_stats(args.channel))
                print(f"{index:04d} ok={ok_count} miss={miss_count} {status}")

            time.sleep(args.interval)

        print("final " + summarize_ip_stats(ip_stats(args.channel)))
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()

