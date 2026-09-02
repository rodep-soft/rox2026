#!/usr/bin/env python3
"""
Step-by-step RobStride EduLite 05 CAN lab.

EduLite 05 default private protocol:
  - CAN bitrate: 1 Mbps
  - Frame type: 29-bit extended CAN
  - New/default motor ID is often 0x7F, but scan first.

Recommended order:
  python3 robstride_edulite05_lab.py listen --seconds 5
  python3 robstride_edulite05_lab.py scan
  python3 robstride_edulite05_lab.py probe --motor-id 0x7F
  python3 robstride_edulite05_lab.py op-jog --motor-id 0x7F --speed 1.0 --duration 1.0
"""

import argparse
import math
import struct
import time
from typing import Iterable, Optional

import can


COMM_GET_ID = 0x00
COMM_CONTROL = 0x01
COMM_FEEDBACK = 0x02
COMM_ENABLE = 0x03
COMM_STOP = 0x04
COMM_READ = 0x11
COMM_WRITE = 0x12
COMM_ACTIVE_REPORT = 0x18

PARAM_RUN_MODE = 0x7005
PARAM_IQ_REF = 0x7006
PARAM_SPD_REF = 0x700A
PARAM_LIMIT_TORQUE = 0x700B
PARAM_LIMIT_CUR = 0x7018
PARAM_MECH_POS = 0x7019
PARAM_IQF = 0x701A
PARAM_MECH_VEL = 0x701B
PARAM_VBUS = 0x701C
PARAM_ACC_RAD = 0x7022
PARAM_CAN_TIMEOUT = 0x7028

RUN_MODE_OPERATION = 0
RUN_MODE_VELOCITY = 2
RUN_MODE_CURRENT = 4

P_MIN = -4.0 * math.pi
P_MAX = 4.0 * math.pi
V_MIN = -50.0
V_MAX = 50.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -6.0
T_MAX = 6.0

MODE_NAMES = {
    0: "reset",
    1: "calibration",
    2: "run",
}

FAULT_NAMES = {
    0: "undervoltage",
    1: "phase_overcurrent",
    2: "overtemperature",
    3: "magnetic_encoder",
    4: "stall_overload",
    5: "uncalibrated",
}

PARAM_NAMES = {
    PARAM_RUN_MODE: ("run_mode", "u8"),
    PARAM_IQ_REF: ("iq_ref", "float"),
    PARAM_SPD_REF: ("spd_ref", "float"),
    PARAM_LIMIT_TORQUE: ("limit_torque", "float"),
    PARAM_LIMIT_CUR: ("limit_cur", "float"),
    PARAM_MECH_POS: ("mechPos", "float"),
    PARAM_IQF: ("iqf", "float"),
    PARAM_MECH_VEL: ("mechVel", "float"),
    PARAM_VBUS: ("VBUS", "float"),
    PARAM_ACC_RAD: ("acc_rad", "float"),
    PARAM_CAN_TIMEOUT: ("canTimeout", "u32"),
}


def parse_int(value: str) -> int:
    return int(value, 0)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def float_to_uint(value: float, lower: float, upper: float) -> int:
    value = clamp(value, lower, upper)
    return int((value - lower) * 65535.0 / (upper - lower))


def uint_to_float(value: int, lower: float, upper: float) -> float:
    return value * (upper - lower) / 65535.0 + lower


def make_ext_id(comm_type: int, target_id: int, data2: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def ext_parts(arb_id: int) -> tuple[int, int, int]:
    comm_type = (arb_id >> 24) & 0x1F
    data2 = (arb_id >> 8) & 0xFFFF
    dest = arb_id & 0xFF
    return comm_type, data2, dest


def hex_data(data: Iterable[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def open_bus(args: argparse.Namespace) -> can.BusABC:
    return can.interface.Bus(
        channel=args.channel,
        interface="socketcan",
        bitrate=args.bitrate,
    )


def send_ext(
    bus: Optional[can.BusABC],
    arb_id: int,
    data: bytes,
    label: str,
    dry_run: bool,
) -> None:
    print(f"tx {label:<16} EXT id=0x{arb_id:X} data={hex_data(data)}")
    if dry_run:
        return
    if bus is None:
        raise RuntimeError("bus is not open")
    bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))


def decode_faults(bits: int) -> str:
    names = [name for bit, name in FAULT_NAMES.items() if bits & (1 << bit)]
    return ",".join(names) if names else "none"


def decode_feedback(msg: can.Message) -> Optional[str]:
    if not msg.is_extended_id or len(msg.data) < 8:
        return None

    comm_type, data2, dest = ext_parts(msg.arbitration_id)
    data = bytes(msg.data)

    if comm_type == COMM_GET_ID and dest == 0xFE:
        motor_id = data2 & 0xFF
        return f"RobStride get-id reply: motor_id=0x{motor_id:02X} mcu_uid={hex_data(data)}"

    if comm_type in (COMM_FEEDBACK, COMM_ACTIVE_REPORT, COMM_ENABLE, COMM_STOP):
        motor_id = data2 & 0xFF
        fault_bits = (data2 >> 8) & 0x3F
        mode = (data2 >> 14) & 0x03
        pos_raw = int.from_bytes(data[0:2], "big")
        vel_raw = int.from_bytes(data[2:4], "big")
        tq_raw = int.from_bytes(data[4:6], "big")
        temp_raw = int.from_bytes(data[6:8], "big")
        pos = uint_to_float(pos_raw, P_MIN, P_MAX)
        vel = uint_to_float(vel_raw, V_MIN, V_MAX)
        torque = uint_to_float(tq_raw, T_MIN, T_MAX)
        temp = temp_raw / 10.0
        return (
            f"RobStride feedback: comm=0x{comm_type:X} motor=0x{motor_id:02X} dest=0x{dest:02X} "
            f"mode={MODE_NAMES.get(mode, mode)} faults={decode_faults(fault_bits)} "
            f"pos={pos:+.3f}rad vel={vel:+.3f}rad/s torque={torque:+.3f}Nm temp={temp:.1f}C"
        )

    if comm_type == COMM_READ:
        index = int.from_bytes(data[0:2], "little")
        name, kind = PARAM_NAMES.get(index, (f"0x{index:04X}", "raw"))
        value_bytes = data[4:8]
        if kind == "float":
            value = struct.unpack("<f", value_bytes)[0]
        elif kind == "u8":
            value = value_bytes[0]
        elif kind == "u32":
            value = int.from_bytes(value_bytes, "little")
        else:
            value = hex_data(value_bytes)
        return f"RobStride read: {name}= {value}"

    return None


def recv_print(bus: can.BusABC, seconds: float, only_robstride: bool = False) -> list[int]:
    deadline = time.monotonic() + seconds
    found_ids = []
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue
        decoded = decode_feedback(msg)
        if decoded is None and only_robstride:
            continue
        frame_kind = "EXT" if msg.is_extended_id else "STD"
        print(
            f"rx {frame_kind} id=0x{msg.arbitration_id:X} dlc={msg.dlc} "
            f"data={hex_data(msg.data)}"
        )
        if decoded is not None:
            print(f"   {decoded}")
            comm_type, data2, dest = ext_parts(msg.arbitration_id)
            if comm_type in (COMM_GET_ID, COMM_FEEDBACK, COMM_ACTIVE_REPORT, COMM_ENABLE, COMM_STOP):
                found_ids.append(data2 & 0xFF)
    return found_ids


def get_id(bus: Optional[can.BusABC], args: argparse.Namespace, motor_id: int) -> None:
    arb_id = make_ext_id(COMM_GET_ID, motor_id, args.host_id)
    send_ext(bus, arb_id, bytes(8), f"get-id 0x{motor_id:02X}", args.dry_run)


def enable_motor(bus: Optional[can.BusABC], args: argparse.Namespace) -> None:
    arb_id = make_ext_id(COMM_ENABLE, args.motor_id, args.host_id)
    send_ext(bus, arb_id, bytes(8), "enable", args.dry_run)


def stop_motor(bus: Optional[can.BusABC], args: argparse.Namespace, clear_fault: bool = False) -> None:
    data = bytearray(8)
    data[0] = 1 if clear_fault else 0
    arb_id = make_ext_id(COMM_STOP, args.motor_id, args.host_id)
    send_ext(bus, arb_id, bytes(data), "stop", args.dry_run)


def read_param(bus: Optional[can.BusABC], args: argparse.Namespace, index: int) -> None:
    data = bytearray(8)
    data[0:2] = index.to_bytes(2, "little")
    name = PARAM_NAMES.get(index, (f"0x{index:04X}",))[0]
    arb_id = make_ext_id(COMM_READ, args.motor_id, args.host_id)
    send_ext(bus, arb_id, bytes(data), f"read {name}", args.dry_run)


def write_u8(bus: Optional[can.BusABC], args: argparse.Namespace, index: int, value: int, label: str) -> None:
    data = bytearray(8)
    data[0:2] = index.to_bytes(2, "little")
    data[4] = value & 0xFF
    arb_id = make_ext_id(COMM_WRITE, args.motor_id, args.host_id)
    send_ext(bus, arb_id, bytes(data), label, args.dry_run)


def write_u32(bus: Optional[can.BusABC], args: argparse.Namespace, index: int, value: int, label: str) -> None:
    data = bytearray(8)
    data[0:2] = index.to_bytes(2, "little")
    data[4:8] = int(value).to_bytes(4, "little", signed=False)
    arb_id = make_ext_id(COMM_WRITE, args.motor_id, args.host_id)
    send_ext(bus, arb_id, bytes(data), label, args.dry_run)


def write_float(bus: Optional[can.BusABC], args: argparse.Namespace, index: int, value: float, label: str) -> None:
    data = bytearray(8)
    data[0:2] = index.to_bytes(2, "little")
    data[4:8] = struct.pack("<f", value)
    arb_id = make_ext_id(COMM_WRITE, args.motor_id, args.host_id)
    send_ext(bus, arb_id, bytes(data), label, args.dry_run)


def control_data(position: float, velocity: float, kp: float, kd: float) -> bytes:
    data = bytearray(8)
    data[0:2] = float_to_uint(position, P_MIN, P_MAX).to_bytes(2, "big")
    data[2:4] = float_to_uint(velocity, V_MIN, V_MAX).to_bytes(2, "big")
    data[4:6] = float_to_uint(kp, KP_MIN, KP_MAX).to_bytes(2, "big")
    data[6:8] = float_to_uint(kd, KD_MIN, KD_MAX).to_bytes(2, "big")
    return bytes(data)


def op_control(
    bus: Optional[can.BusABC],
    args: argparse.Namespace,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
) -> None:
    torque_raw = float_to_uint(torque, T_MIN, T_MAX)
    arb_id = make_ext_id(COMM_CONTROL, args.motor_id, torque_raw)
    send_ext(bus, arb_id, control_data(position, velocity, kp, kd), "op-control", args.dry_run)


def require_confirm(args: argparse.Namespace, description: str) -> None:
    if args.yes or args.dry_run:
        return
    print(description)
    input("Press Enter to send, or Ctrl+C to cancel. ")


def cmd_listen(args: argparse.Namespace) -> None:
    bus = open_bus(args)
    try:
        print(f"Listening on {args.channel} at {args.bitrate} bps for {args.seconds}s...")
        recv_print(bus, args.seconds, only_robstride=args.only_robstride)
    finally:
        bus.shutdown()


def cmd_scan(args: argparse.Namespace) -> None:
    bus = None if args.dry_run else open_bus(args)
    found = set()
    try:
        print(
            f"Scanning RobStride IDs 0x{args.start:02X}..0x{args.end:02X} "
            f"on {args.channel} at {args.bitrate} bps..."
        )
        for motor_id in range(args.start, args.end + 1):
            get_id(bus, args, motor_id)
            if bus is not None:
                found.update(recv_print(bus, args.gap, only_robstride=True))
            else:
                time.sleep(args.gap)

        if bus is not None:
            found.update(recv_print(bus, args.listen_after, only_robstride=True))

        if found:
            print("Found motor IDs: " + ", ".join(f"0x{motor_id:02X}" for motor_id in sorted(found)))
        else:
            print("No RobStride private-protocol replies found.")
    finally:
        if bus is not None:
            bus.shutdown()


def cmd_probe(args: argparse.Namespace) -> None:
    bus = None if args.dry_run else open_bus(args)
    try:
        get_id(bus, args, args.motor_id)
        if bus is not None:
            recv_print(bus, args.gap, only_robstride=False)
        for index in (PARAM_RUN_MODE, PARAM_VBUS, PARAM_MECH_POS, PARAM_MECH_VEL, PARAM_IQF, PARAM_CAN_TIMEOUT):
            read_param(bus, args, index)
            if bus is not None:
                recv_print(bus, args.gap, only_robstride=False)
            else:
                time.sleep(args.gap)
        if bus is not None:
            recv_print(bus, args.listen_after, only_robstride=False)
    finally:
        if bus is not None:
            bus.shutdown()


def cmd_enable(args: argparse.Namespace) -> None:
    require_confirm(args, f"About to enable RobStride motor 0x{args.motor_id:02X}.")
    bus = None if args.dry_run else open_bus(args)
    try:
        enable_motor(bus, args)
        if bus is not None:
            recv_print(bus, args.listen_after, only_robstride=False)
    finally:
        if bus is not None:
            bus.shutdown()


def cmd_stop(args: argparse.Namespace) -> None:
    bus = None if args.dry_run else open_bus(args)
    try:
        stop_motor(bus, args, clear_fault=args.clear_fault)
        if bus is not None:
            recv_print(bus, args.listen_after, only_robstride=False)
    finally:
        if bus is not None:
            bus.shutdown()


def cmd_op_jog(args: argparse.Namespace) -> None:
    require_confirm(
        args,
        (
            f"About to command RobStride 0x{args.motor_id:02X}: "
            f"operation mode velocity={args.speed} rad/s, kd={args.kd}, duration={args.duration}s."
        ),
    )
    bus = None if args.dry_run else open_bus(args)
    try:
        enable_motor(bus, args)
        if bus is not None:
            recv_print(bus, args.gap, only_robstride=False)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            op_control(bus, args, position=0.0, velocity=args.speed, kp=0.0, kd=args.kd, torque=0.0)
            if bus is None:
                time.sleep(args.period)
            else:
                recv_print(bus, min(args.period, 0.05), only_robstride=False)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            op_control(bus, args, position=0.0, velocity=0.0, kp=0.0, kd=args.kd, torque=0.0)
            time.sleep(args.gap)
            stop_motor(bus, args)
        finally:
            if bus is not None:
                bus.shutdown()


def cmd_vel_jog(args: argparse.Namespace) -> None:
    require_confirm(
        args,
        (
            f"About to command RobStride 0x{args.motor_id:02X}: "
            f"velocity mode speed={args.speed} rad/s, limit_cur={args.current_limit}A, duration={args.duration}s."
        ),
    )
    bus = None if args.dry_run else open_bus(args)
    try:
        write_u8(bus, args, PARAM_RUN_MODE, RUN_MODE_VELOCITY, "run_mode=vel")
        if bus is not None:
            recv_print(bus, args.gap, only_robstride=False)
        enable_motor(bus, args)
        if bus is not None:
            recv_print(bus, args.gap, only_robstride=False)
        write_float(bus, args, PARAM_LIMIT_CUR, args.current_limit, "limit_cur")
        if bus is not None:
            recv_print(bus, args.gap, only_robstride=False)
        write_float(bus, args, PARAM_ACC_RAD, args.accel, "acc_rad")
        if bus is not None:
            recv_print(bus, args.gap, only_robstride=False)

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            write_float(bus, args, PARAM_SPD_REF, args.speed, "spd_ref")
            if bus is None:
                time.sleep(args.period)
            else:
                recv_print(bus, min(args.period, 0.05), only_robstride=False)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            write_float(bus, args, PARAM_SPD_REF, 0.0, "spd_ref=0")
            time.sleep(args.gap)
            stop_motor(bus, args)
        finally:
            if bus is not None:
                bus.shutdown()


def cmd_wiggle(args: argparse.Namespace) -> None:
    require_confirm(
        args,
        (
            f"About to wiggle RobStride 0x{args.motor_id:02X}: "
            f"+/-{args.speed} rad/s, half_period={args.half_period}s."
        ),
    )
    bus = None if args.dry_run else open_bus(args)
    start = time.monotonic()
    last_phase = None
    try:
        enable_motor(bus, args)
        deadline = None if args.duration <= 0 else start + args.duration
        while deadline is None or time.monotonic() < deadline:
            elapsed = time.monotonic() - start
            phase = int(elapsed / args.half_period) % 2
            speed = args.speed if phase == 0 else -args.speed
            if phase != last_phase:
                print(f"phase={'forward' if phase == 0 else 'reverse'} speed={speed:+.3f}rad/s")
                last_phase = phase
            op_control(bus, args, position=0.0, velocity=speed, kp=0.0, kd=args.kd, torque=0.0)
            if bus is None:
                time.sleep(args.period)
            else:
                recv_print(bus, min(args.period, 0.05), only_robstride=False)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            op_control(bus, args, position=0.0, velocity=0.0, kp=0.0, kd=args.kd, torque=0.0)
            time.sleep(args.gap)
            stop_motor(bus, args)
        finally:
            if bus is not None:
                bus.shutdown()


def cmd_set_timeout(args: argparse.Namespace) -> None:
    bus = None if args.dry_run else open_bus(args)
    ticks = int(args.seconds * 20000)
    try:
        write_u32(bus, args, PARAM_CAN_TIMEOUT, ticks, f"timeout={args.seconds}s")
        if bus is not None:
            recv_print(bus, args.listen_after, only_robstride=False)
    finally:
        if bus is not None:
            bus.shutdown()


def add_bus_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--host-id", type=parse_int, default=0xFD)
    parser.add_argument("--dry-run", action="store_true")


def add_motor_args(parser: argparse.ArgumentParser) -> None:
    add_bus_args(parser)
    parser.add_argument("--motor-id", type=parse_int, default=0x7F)


def add_motion_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--period", type=float, default=0.02)
    parser.add_argument("--gap", type=float, default=0.05)
    parser.add_argument("--yes", action="store_true")


def main() -> None:
    parser = argparse.ArgumentParser(description="RobStride EduLite 05 step-by-step CAN tool.")
    sub = parser.add_subparsers(dest="command", required=True)

    listen = sub.add_parser("listen", help="Listen and decode RobStride frames")
    add_bus_args(listen)
    listen.add_argument("--seconds", type=float, default=5.0)
    listen.add_argument("--only-robstride", action="store_true")
    listen.set_defaults(func=cmd_listen)

    scan = sub.add_parser("scan", help="Non-moving get-ID scan")
    add_bus_args(scan)
    scan.add_argument("--start", type=parse_int, default=0x01)
    scan.add_argument("--end", type=parse_int, default=0x7F)
    scan.add_argument("--gap", type=float, default=0.02)
    scan.add_argument("--listen-after", type=float, default=0.5)
    scan.set_defaults(func=cmd_scan)

    probe = sub.add_parser("probe", help="Non-moving reads: ID, VBUS, position, velocity")
    add_motor_args(probe)
    probe.add_argument("--gap", type=float, default=0.08)
    probe.add_argument("--listen-after", type=float, default=0.5)
    probe.set_defaults(func=cmd_probe)

    enable = sub.add_parser("enable", help="Enable motor and listen for feedback")
    add_motor_args(enable)
    enable.add_argument("--listen-after", type=float, default=0.5)
    enable.add_argument("--yes", action="store_true")
    enable.set_defaults(func=cmd_enable)

    stop = sub.add_parser("stop", help="Stop motor")
    add_motor_args(stop)
    stop.add_argument("--clear-fault", action="store_true")
    stop.add_argument("--listen-after", type=float, default=0.5)
    stop.set_defaults(func=cmd_stop)

    op = sub.add_parser("op-jog", help="Default operation-mode velocity jog")
    add_motor_args(op)
    add_motion_args(op)
    op.add_argument("--speed", type=float, default=1.0, help="rad/s, default: 1.0")
    op.add_argument("--kd", type=float, default=1.0)
    op.set_defaults(func=cmd_op_jog)

    vel = sub.add_parser("vel-jog", help="Velocity-mode jog using run_mode=2 and spd_ref")
    add_motor_args(vel)
    add_motion_args(vel)
    vel.add_argument("--speed", type=float, default=1.0, help="rad/s, default: 1.0")
    vel.add_argument("--current-limit", type=float, default=1.0, help="A, default: 1.0")
    vel.add_argument("--accel", type=float, default=5.0, help="rad/s^2, default: 5.0")
    vel.set_defaults(func=cmd_vel_jog)

    wiggle = sub.add_parser("wiggle", help="Alternate +speed/-speed for connector tests")
    add_motor_args(wiggle)
    add_motion_args(wiggle)
    wiggle.add_argument("--speed", type=float, default=1.0, help="rad/s, default: 1.0")
    wiggle.add_argument("--half-period", type=float, default=0.3)
    wiggle.add_argument("--kd", type=float, default=1.0)
    wiggle.set_defaults(func=cmd_wiggle)

    timeout = sub.add_parser("set-timeout", help="Set volatile CAN timeout threshold")
    add_motor_args(timeout)
    timeout.add_argument("--seconds", type=float, default=0.2)
    timeout.add_argument("--listen-after", type=float, default=0.5)
    timeout.set_defaults(func=cmd_set_timeout)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

