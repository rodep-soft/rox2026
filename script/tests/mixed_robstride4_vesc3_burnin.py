#!/usr/bin/env python3
"""
Mixed CAN burn-in drive:
  - RobStride EduLite 05 IDs 0x20..0x23: alternating velocity command
  - VESC-compatible ESC IDs 0x32..0x34: one-direction current command

The script is intended for a long CAN stability test on can0 at 1 Mbps.
It never commands the forbidden RobStride IDs 0x28 or 0x38.

Examples:
  python3 mixed_robstride4_vesc3_burnin.py --yes
  python3 mixed_robstride4_vesc3_burnin.py --duration 300 --yes
  python3 mixed_robstride4_vesc3_burnin.py --esc-current 10 --rs-speed 1.0 --yes
  python3 mixed_robstride4_vesc3_burnin.py --esc-reverse-ids 0x32 --yes
  python3 mixed_robstride4_vesc3_burnin.py --rs-stagger --yes
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import time


RS_COMM_CONTROL = 0x01
RS_COMM_ENABLE = 0x03
RS_COMM_STOP = 0x04

VESC_PACKET_SET_CURRENT = 1
VESC_PACKET_SET_RPM = 3
VESC_PACKET_STATUS = 9
VESC_PACKET_STATUS_4 = 16

DEFAULT_RS_IDS = (0x20, 0x21, 0x22, 0x23)
FORBIDDEN_RS_IDS = (0x28, 0x38)
DEFAULT_ESC_IDS = (0x32, 0x33, 0x34)

RS_P_MIN = -4.0 * math.pi
RS_P_MAX = 4.0 * math.pi
RS_V_MIN = -50.0
RS_V_MAX = 50.0
RS_KP_MIN = 0.0
RS_KP_MAX = 500.0
RS_KD_MIN = 0.0
RS_KD_MAX = 5.0
RS_T_MIN = -6.0
RS_T_MAX = 6.0

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
        raise argparse.ArgumentTypeError("at least one ID is required")
    for item in ids:
        if not 0 <= item <= 0xFF:
            raise argparse.ArgumentTypeError(f"invalid ID: 0x{item:X}")
    return ids


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def float_to_uint(value: float, lower: float, upper: float) -> int:
    value = clamp(value, lower, upper)
    return int((value - lower) * 65535.0 / (upper - lower))


def hex_data(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def open_bus(args):
    can_module = load_can_module()
    return can_module.interface.Bus(channel=args.channel, interface="socketcan", bitrate=args.bitrate)


def make_rs_ext_id(comm_type: int, target_id: int, data2: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def make_vesc_ext_id(packet_id: int, controller_id: int) -> int:
    return ((packet_id & 0xFF) << 8) | (controller_id & 0xFF)


def send_ext(bus, arb_id: int, data: bytes, args, label: str) -> bool:
    can_module = load_can_module()
    if args.dry_run:
        print(f"dry tx {label:<22} EXT id=0x{arb_id:X} data={hex_data(data)}")
        return True

    msg = can_module.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
    for attempt in range(args.send_retries + 1):
        try:
            bus.send(msg, timeout=args.send_timeout)
            return True
        except can_module.CanOperationError as exc:
            if attempt >= args.send_retries:
                print(f"CAN TX FAILED {label} id=0x{arb_id:X}: {exc}")
                return False
            time.sleep(0.005)
    return False


def rs_control_data(position: float, velocity: float, kp: float, kd: float) -> bytes:
    data = bytearray(8)
    data[0:2] = float_to_uint(position, RS_P_MIN, RS_P_MAX).to_bytes(2, "big")
    data[2:4] = float_to_uint(velocity, RS_V_MIN, RS_V_MAX).to_bytes(2, "big")
    data[4:6] = float_to_uint(kp, RS_KP_MIN, RS_KP_MAX).to_bytes(2, "big")
    data[6:8] = float_to_uint(kd, RS_KD_MIN, RS_KD_MAX).to_bytes(2, "big")
    return bytes(data)


def rs_op_control(bus, args, motor_id: int, velocity: float, torque: float = 0.0) -> bool:
    torque_raw = float_to_uint(torque, RS_T_MIN, RS_T_MAX)
    arb_id = make_rs_ext_id(RS_COMM_CONTROL, motor_id, torque_raw)
    data = rs_control_data(position=0.0, velocity=velocity, kp=0.0, kd=args.rs_kd)
    return send_ext(bus, arb_id, data, args, f"rs 0x{motor_id:02X} vel={velocity:+.2f}")


def rs_enable(bus, args, motor_id: int) -> bool:
    arb_id = make_rs_ext_id(RS_COMM_ENABLE, motor_id, args.host_id)
    return send_ext(bus, arb_id, bytes(8), args, f"rs enable 0x{motor_id:02X}")


def rs_stop(bus, args, motor_id: int) -> bool:
    arb_id = make_rs_ext_id(RS_COMM_STOP, motor_id, args.host_id)
    return send_ext(bus, arb_id, bytes(8), args, f"rs stop 0x{motor_id:02X}")


def vesc_set_current(bus, args, controller_id: int, current_a: float) -> bool:
    raw = int(current_a * 1000.0)
    arb_id = make_vesc_ext_id(VESC_PACKET_SET_CURRENT, controller_id)
    data = raw.to_bytes(4, "big", signed=True)
    return send_ext(bus, arb_id, data, args, f"esc 0x{controller_id:02X} cur={current_a:+.1f}A")


def vesc_set_rpm(bus, args, controller_id: int, rpm: int) -> bool:
    arb_id = make_vesc_ext_id(VESC_PACKET_SET_RPM, controller_id)
    data = int(rpm).to_bytes(4, "big", signed=True)
    return send_ext(bus, arb_id, data, args, f"esc 0x{controller_id:02X} rpm={rpm:+d}")


def sign_for(controller_id: int, reverse_ids: set[int]) -> int:
    return -1 if controller_id in reverse_ids else 1


def rs_velocity_for(args, index: int, elapsed: float) -> float:
    phase_elapsed = elapsed
    if args.rs_stagger:
        phase_elapsed += index * args.rs_half_period / max(1, len(args.rs_ids))
    sign = 1.0 if int(phase_elapsed / args.rs_half_period) % 2 == 0 else -1.0
    return sign * abs(args.rs_speed)


def send_drive_frame(bus, args, elapsed: float) -> bool:
    ok = True

    for index, motor_id in enumerate(args.rs_ids):
        velocity = rs_velocity_for(args, index, elapsed)
        ok = rs_op_control(bus, args, motor_id, velocity) and ok
        time.sleep(args.inter_frame_gap)

    for controller_id in args.esc_ids:
        current = sign_for(controller_id, args.esc_reverse_ids) * args.esc_current
        ok = vesc_set_current(bus, args, controller_id, current) and ok
        time.sleep(args.inter_frame_gap)

    return ok


def send_all_zero_and_stop(bus, args) -> None:
    print("Sending ESC zero current/rpm and RobStride zero velocity/stop...")

    for _ in range(args.stop_zero_repeats):
        for controller_id in args.esc_ids:
            vesc_set_current(bus, args, controller_id, 0.0)
            time.sleep(args.inter_frame_gap)
        for motor_id in args.rs_ids:
            rs_op_control(bus, args, motor_id, 0.0)
            time.sleep(args.inter_frame_gap)
        time.sleep(0.02)

    for controller_id in args.esc_ids:
        vesc_set_rpm(bus, args, controller_id, 0)
        time.sleep(args.inter_frame_gap)
    for motor_id in args.rs_ids:
        rs_stop(bus, args, motor_id)
        time.sleep(0.03)


def drain_rx(bus, args) -> tuple[int, dict[int, int], dict[int, int]]:
    count = 0
    esc_status = {}
    rs_reply = {}

    while count < args.drain_rx_frames:
        msg = bus.recv(timeout=0.0)
        if msg is None:
            break
        count += 1
        if not msg.is_extended_id:
            continue

        packet = (msg.arbitration_id >> 8) & 0xFF
        low_id = msg.arbitration_id & 0xFF
        if packet in (VESC_PACKET_STATUS, VESC_PACKET_STATUS_4) and low_id in args.esc_ids:
            esc_status[low_id] = esc_status.get(low_id, 0) + 1

        rs_motor_guess = (msg.arbitration_id >> 8) & 0xFFFF
        if low_id == 0xFE and rs_motor_guess in args.rs_ids:
            rs_reply[rs_motor_guess] = rs_reply.get(rs_motor_guess, 0) + 1

    return count, esc_status, rs_reply


def ip_stats(channel: str) -> str:
    result = subprocess.run(
        ["ip", "-details", "-statistics", "link", "show", channel],
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout


def parse_ip_stats(text: str) -> dict[str, int | str]:
    state_match = re.search(r"can state ([A-Z-]+) \(berr-counter tx (\d+) rx (\d+)\)", text)
    totals_match = re.search(
        r"re-started bus-errors arbit-lost error-warn error-pass bus-off\s+"
        r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
        text,
        re.MULTILINE,
    )

    stats: dict[str, int | str] = {
        "state": "UNKNOWN",
        "txerr": -1,
        "rxerr": -1,
        "restart": -1,
        "warn": -1,
        "passive": -1,
        "busoff": -1,
    }
    if state_match:
        stats["state"] = state_match.group(1)
        stats["txerr"] = int(state_match.group(2))
        stats["rxerr"] = int(state_match.group(3))
    if totals_match:
        stats["restart"] = int(totals_match.group(1))
        stats["warn"] = int(totals_match.group(4))
        stats["passive"] = int(totals_match.group(5))
        stats["busoff"] = int(totals_match.group(6))
    return stats


def stats_summary(stats: dict[str, int | str]) -> str:
    return (
        f"state={stats['state']} txerr={stats['txerr']} rxerr={stats['rxerr']} "
        f"restart={stats['restart']} warn={stats['warn']} pass={stats['passive']} busoff={stats['busoff']}"
    )


def stats_bad_or_changed(stats: dict[str, int | str], baseline: dict[str, int | str]) -> bool:
    if stats["state"] != "ERROR-ACTIVE":
        return True
    if stats["txerr"] != 0 or stats["rxerr"] != 0:
        return True
    for key in ("restart", "warn", "passive", "busoff"):
        if isinstance(stats[key], int) and isinstance(baseline[key], int) and stats[key] > baseline[key]:
            return True
    return False


def validate_args(args) -> None:
    if len(args.rs_ids) != len(set(args.rs_ids)):
        raise SystemExit("Duplicate RobStride IDs in --rs-ids.")
    if len(args.esc_ids) != len(set(args.esc_ids)):
        raise SystemExit("Duplicate ESC IDs in --esc-ids.")
    blocked = sorted(set(args.rs_ids) & set(args.forbidden_rs_ids))
    if blocked:
        ids = ", ".join(f"0x{x:02X}" for x in blocked)
        raise SystemExit(f"Refusing to command forbidden RobStride ID(s): {ids}")
    if not set(args.esc_reverse_ids).issubset(set(args.esc_ids)):
        raise SystemExit("--esc-reverse-ids must be a subset of --esc-ids.")
    if args.period <= 0 or args.inter_frame_gap < 0 or args.rs_half_period <= 0:
        raise SystemExit("periods and gaps must be positive.")
    if not args.allow_high and abs(args.esc_current) > 3.0:
        raise SystemExit("Refusing ESC current > 3A without --allow-high.")
    if abs(args.rs_speed) > RS_V_MAX:
        raise SystemExit(f"RobStride speed must be within +/-{RS_V_MAX} rad/s.")


def confirm_if_needed(args) -> None:
    if args.yes or args.dry_run:
        return

    duration = "until Ctrl+C" if args.duration <= 0 else f"{args.duration}s"
    rs_ids = ", ".join(f"0x{x:02X}" for x in args.rs_ids)
    esc_ids = ", ".join(f"0x{x:02X}" for x in args.esc_ids)
    forbidden = ", ".join(f"0x{x:02X}" for x in args.forbidden_rs_ids)
    print(f"RobStride alternate IDs: {rs_ids}")
    print(f"ESC one-direction IDs:   {esc_ids}")
    print(f"Forbidden RobStride IDs: {forbidden}")
    print(f"RobStride speed: +/-{abs(args.rs_speed)} rad/s, half-period {args.rs_half_period}s")
    print(f"ESC current: {args.esc_current:+.1f}A, duration {duration}")
    print("Make sure the robot is lifted and power can be cut immediately.")
    input("Press Enter to start, or Ctrl+C to cancel. ")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run RobStride4 alternate + VESC3 constant-current CAN burn-in.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--host-id", type=parse_int, default=0xFD)
    parser.add_argument("--rs-ids", type=parse_ids, default=list(DEFAULT_RS_IDS))
    parser.add_argument("--forbidden-rs-ids", type=parse_ids, default=list(FORBIDDEN_RS_IDS))
    parser.add_argument("--esc-ids", type=parse_ids, default=list(DEFAULT_ESC_IDS))
    parser.add_argument("--esc-reverse-ids", type=parse_ids, default=[])
    parser.add_argument("--rs-speed", type=float, default=1.0, help="RobStride velocity in rad/s")
    parser.add_argument("--rs-half-period", type=float, default=0.4)
    parser.add_argument("--rs-kd", type=float, default=1.0)
    parser.add_argument("--rs-stagger", action="store_true", help="Offset RobStride phases between motors")
    parser.add_argument("--esc-current", type=float, default=10.0, help="VESC motor current in A")
    parser.add_argument("--period", type=float, default=0.05, help="full command-loop period")
    parser.add_argument("--inter-frame-gap", type=float, default=0.004)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl+C")
    parser.add_argument("--stats-period", type=float, default=1.0)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--drain-rx-frames", type=int, default=128)
    parser.add_argument("--send-timeout", type=float, default=0.01)
    parser.add_argument("--send-retries", type=int, default=0)
    parser.add_argument("--max-consecutive-tx-fails", type=int, default=5)
    parser.add_argument("--stop-on-error", action="store_true", help="Stop the test when CAN errors are detected")
    parser.add_argument("--stop-zero-repeats", type=int, default=5)
    parser.add_argument("--allow-high", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    args.esc_reverse_ids = set(args.esc_reverse_ids)
    validate_args(args)
    confirm_if_needed(args)

    bus = open_bus(args)
    start = time.monotonic()
    next_print = start
    next_stats = start
    loop_count = 0
    tx_failures = 0
    consecutive_tx_failures = 0
    total_rx_drained = 0
    total_esc_status = {controller_id: 0 for controller_id in args.esc_ids}
    total_rs_reply = {motor_id: 0 for motor_id in args.rs_ids}
    baseline_stats = parse_ip_stats(ip_stats(args.channel))

    try:
        print("start " + stats_summary(baseline_stats))
        print("Enabling RobStride motors...")
        for motor_id in args.rs_ids:
            if not rs_enable(bus, args, motor_id):
                raise SystemExit(f"Failed to enable RobStride 0x{motor_id:02X}.")
            time.sleep(0.03)

        print("Running mixed burn-in. Ctrl+C stops commanded IDs.")
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            loop_started = time.monotonic()
            elapsed = loop_started - start
            loop_count += 1

            if send_drive_frame(bus, args, elapsed):
                consecutive_tx_failures = 0
            else:
                tx_failures += 1
                consecutive_tx_failures += 1
                if consecutive_tx_failures >= args.max_consecutive_tx_fails:
                    print(f"CAN TX failure threshold reached: consecutive={consecutive_tx_failures}")
                    if args.stop_on_error:
                        break
                    time.sleep(0.2)

            rx_count, esc_seen, rs_seen = drain_rx(bus, args)
            total_rx_drained += rx_count
            for controller_id, count in esc_seen.items():
                total_esc_status[controller_id] = total_esc_status.get(controller_id, 0) + count
            for motor_id, count in rs_seen.items():
                total_rs_reply[motor_id] = total_rs_reply.get(motor_id, 0) + count

            now = time.monotonic()
            if now >= next_stats:
                stats = parse_ip_stats(ip_stats(args.channel))
                if stats_bad_or_changed(stats, baseline_stats):
                    print("CAN WARN " + stats_summary(stats))
                    if args.stop_on_error:
                        break
                next_stats = now + args.stats_period

            if now >= next_print:
                rs_phase = []
                for index, motor_id in enumerate(args.rs_ids):
                    rs_phase.append(f"0x{motor_id:02X}:{rs_velocity_for(args, index, elapsed):+.1f}")
                esc_phase = []
                for controller_id in args.esc_ids:
                    current = sign_for(controller_id, args.esc_reverse_ids) * args.esc_current
                    esc_phase.append(f"0x{controller_id:02X}:{current:+.1f}A")
                print(
                    f"t={elapsed:7.1f}s loop={loop_count} tx_fail={tx_failures} rx={total_rx_drained} "
                    f"RS {' '.join(rs_phase)} | ESC {' '.join(esc_phase)}"
                )
                next_print = now + args.print_period

            sleep_time = args.period - (time.monotonic() - loop_started)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            send_all_zero_and_stop(bus, args)
        finally:
            print("final " + stats_summary(parse_ip_stats(ip_stats(args.channel))))
            esc_summary = " ".join(f"0x{k:02X}:{v}" for k, v in sorted(total_esc_status.items()))
            rs_summary = " ".join(f"0x{k:02X}:{v}" for k, v in sorted(total_rs_reply.items()))
            print(f"rx summary ESC_STATUS={esc_summary} RS_REPLY={rs_summary} drained={total_rx_drained}")
            bus.shutdown()


if __name__ == "__main__":
    main()

