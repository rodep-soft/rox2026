#!/usr/bin/env python3
"""
Kick-start multiple VESC-compatible ESCs with current, then hand off to SET_RPM.

Each ESC has its own kick/rpm state, so one ESC can re-kick without forcing the
other ESC out of RPM control.

Examples:
  python3 vesc_multi_rpm_kickstart_1mbps.py --ids 0x33,0x34 --target-rpm 5000 --allow-high --yes
  python3 vesc_multi_rpm_kickstart_1mbps.py --ids 0x33,0x34 --target-erpm 35000 --kick-current 8 --allow-high --yes
  python3 vesc_multi_rpm_kickstart_1mbps.py --ids 0x33,0x34 --reverse-ids 0x34 --target-rpm 5000 --allow-high --yes
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass


PACKET_SET_CURRENT = 1
PACKET_SET_RPM = 3
PACKET_STATUS = 9

can = None


@dataclass
class EscState:
    phase: str = "kick"
    phase_start: float = 0.0
    latest_erpm: int = 0
    latest_current: float = 0.0
    latest_duty: float = 0.0
    last_status_time: float | None = None
    rpm_command: int = 0
    rpm_ramp_start_erpm: int = 0
    tx_fail: int = 0


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
    for controller_id in ids:
        if not 0 <= controller_id <= 0xFF:
            raise argparse.ArgumentTypeError(f"invalid ID: 0x{controller_id:X}")
    return ids


def vesc_id(packet_id: int, controller_id: int) -> int:
    return ((packet_id & 0xFF) << 8) | (controller_id & 0xFF)


def be_i16(data: bytes, offset: int) -> int:
    return struct.unpack(">h", data[offset : offset + 2])[0]


def be_i32(data: bytes, offset: int) -> int:
    return struct.unpack(">i", data[offset : offset + 4])[0]


def open_bus(args):
    can_module = load_can_module()
    return can_module.interface.Bus(channel=args.channel, interface="socketcan", bitrate=args.bitrate)


def send_current(bus, args, controller_id: int, current_a: float) -> bool:
    can_module = load_can_module()
    raw = int(current_a * 1000.0)
    arb_id = vesc_id(PACKET_SET_CURRENT, controller_id)
    msg = can_module.Message(
        arbitration_id=arb_id,
        data=raw.to_bytes(4, "big", signed=True),
        is_extended_id=True,
    )
    try:
        bus.send(msg, timeout=args.send_timeout)
        return True
    except can_module.CanOperationError as exc:
        print(f"CAN TX FAILED current id=0x{controller_id:02X} arb=0x{arb_id:X}: {exc}")
        return False


def send_rpm(bus, args, controller_id: int, erpm: int) -> bool:
    can_module = load_can_module()
    arb_id = vesc_id(PACKET_SET_RPM, controller_id)
    msg = can_module.Message(
        arbitration_id=arb_id,
        data=int(erpm).to_bytes(4, "big", signed=True),
        is_extended_id=True,
    )
    try:
        bus.send(msg, timeout=args.send_timeout)
        return True
    except can_module.CanOperationError as exc:
        print(f"CAN TX FAILED rpm id=0x{controller_id:02X} arb=0x{arb_id:X}: {exc}")
        return False


def drain_status(bus, args, states: dict[int, EscState], now: float) -> int:
    count = 0
    while count < args.drain_rx_frames:
        msg = bus.recv(timeout=0.0)
        if msg is None:
            break
        count += 1
        if not msg.is_extended_id:
            continue
        packet = (msg.arbitration_id >> 8) & 0xFF
        controller_id = msg.arbitration_id & 0xFF
        if packet != PACKET_STATUS or controller_id not in states:
            continue
        data = bytes(msg.data)
        if len(data) < 8:
            continue
        state = states[controller_id]
        state.latest_erpm = be_i32(data, 0)
        state.latest_current = be_i16(data, 4) / 10.0
        state.latest_duty = be_i16(data, 6) / 1000.0
        state.last_status_time = now
    return count


def sign_for(controller_id: int, reverse_ids: set[int]) -> int:
    return -1 if controller_id in reverse_ids else 1


def target_erpm(args) -> int:
    if args.target_erpm is not None:
        return args.target_erpm
    return int(round(args.target_rpm * args.pole_pairs))


def ramp_between(start_value: float, target_value: float, elapsed: float, ramp_time: float) -> float:
    if ramp_time <= 0:
        return target_value
    scale = min(1.0, max(0.0, elapsed / ramp_time))
    return start_value + (target_value - start_value) * scale


def safety_check(args) -> None:
    if args.pole_pairs <= 0:
        raise SystemExit("--pole-pairs must be positive")
    if args.period <= 0 or args.inter_frame_gap < 0:
        raise SystemExit("periods and gaps must be positive")
    if len(args.ids) != len(set(args.ids)):
        raise SystemExit("Duplicate IDs in --ids")
    if not set(args.reverse_ids).issubset(set(args.ids)):
        raise SystemExit("--reverse-ids must be a subset of --ids")
    if args.allow_high:
        return
    if abs(args.kick_current) > 3.0:
        raise SystemExit("Refusing --kick-current > 3A without --allow-high.")
    if abs(target_erpm(args)) > 4000:
        raise SystemExit("Refusing target above 4000 ERPM without --allow-high.")


def confirm_if_needed(args, base_target_erpm: int) -> None:
    if args.yes or args.dry_run:
        return
    ids = ", ".join(f"0x{x:02X}" for x in args.ids)
    reverse = ", ".join(f"0x{x:02X}" for x in args.reverse_ids) or "(none)"
    print(f"ESC IDs: {ids}")
    print(f"Reverse IDs: {reverse}")
    print(f"Target: {base_target_erpm:+d} ERPM ({base_target_erpm / args.pole_pairs:+.1f} mechanical RPM if pole_pairs={args.pole_pairs})")
    print(f"Kick current: {args.kick_current:+.1f}A")
    print("Make sure belts/flywheels are guarded and power can be cut immediately.")
    input("Press Enter to start, or Ctrl+C to cancel. ")


def wait_initial_status(bus, args, states: dict[int, EscState]) -> None:
    deadline = time.monotonic() + args.status_timeout
    while time.monotonic() < deadline:
        now = time.monotonic()
        drain_status(bus, args, states, now)
        if all(state.last_status_time is not None for state in states.values()):
            return
        time.sleep(0.01)
    missing = [controller_id for controller_id, state in states.items() if state.last_status_time is None]
    if missing:
        formatted = ", ".join(f"0x{x:02X}" for x in missing)
        print(f"WARNING: no initial STATUS from {formatted}")


def run_one_esc(bus, args, controller_id: int, state: EscState, erpm_target: int, now: float) -> None:
    direction = 1 if erpm_target >= 0 else -1
    speed_abs = abs(state.latest_erpm)
    handoff = abs(args.handoff_erpm)
    rekick = abs(args.rekick_erpm)
    phase_elapsed = now - state.phase_start

    if state.last_status_time is not None and now - state.last_status_time > args.status_timeout:
        print(f"Feedback lost on 0x{controller_id:02X}; forcing kick phase")
        state.phase = "kick"
        state.phase_start = now

    if state.phase == "kick":
        ok = send_current(bus, args, controller_id, direction * abs(args.kick_current))
        if speed_abs >= handoff or phase_elapsed >= args.kick_time:
            state.phase = "rpm"
            state.phase_start = now
            ramp_start_abs = min(max(speed_abs, handoff), abs(erpm_target))
            state.rpm_ramp_start_erpm = int(direction * ramp_start_abs)
            state.rpm_command = state.rpm_ramp_start_erpm
    else:
        if speed_abs < rekick:
            print(f"0x{controller_id:02X} speed below rekick threshold: erpm={state.latest_erpm:+d}. Kicking again.")
            state.phase = "kick"
            state.phase_start = now
            ok = send_current(bus, args, controller_id, direction * abs(args.kick_current))
        else:
            ramped = ramp_between(state.rpm_ramp_start_erpm, erpm_target, phase_elapsed, args.rpm_ramp_time)
            state.rpm_command = int(ramped)
            ok = send_rpm(bus, args, controller_id, state.rpm_command)

    if not ok:
        state.tx_fail += 1


def smooth_stop(bus, args, states: dict[int, EscState]) -> None:
    if args.stop_ramp_time > 0:
        steps = max(1, int(args.stop_ramp_time / max(args.period, 1e-3)))
        print(f"Ramping SET_RPM to zero over {args.stop_ramp_time:.2f}s...")
        for step in range(steps - 1, -1, -1):
            for controller_id, state in states.items():
                send_rpm(bus, args, controller_id, int(state.rpm_command * step / steps))
                time.sleep(args.inter_frame_gap)
            time.sleep(args.period)

    print("Sending zero current/rpm...")
    for _ in range(args.zero_repeats):
        for controller_id in states:
            send_current(bus, args, controller_id, 0.0)
            time.sleep(args.inter_frame_gap)
        time.sleep(args.zero_gap)
    for controller_id in states:
        send_rpm(bus, args, controller_id, 0)
        time.sleep(args.inter_frame_gap)


def main() -> None:
    parser = argparse.ArgumentParser(description="Kick-start multiple VESC ESCs, then run SET_RPM.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--ids", type=parse_ids, default=parse_ids("0x33,0x34"))
    parser.add_argument("--reverse-ids", type=parse_ids, default=[])
    parser.add_argument("--target-rpm", type=float, default=1000.0, help="mechanical RPM target")
    parser.add_argument("--target-erpm", type=int, default=None, help="override target as VESC ERPM")
    parser.add_argument("--pole-pairs", type=int, default=7)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--inter-frame-gap", type=float, default=0.006)
    parser.add_argument("--kick-current", type=float, default=8.0)
    parser.add_argument("--kick-time", type=float, default=1.0)
    parser.add_argument("--handoff-erpm", type=int, default=1500)
    parser.add_argument("--rekick-erpm", type=int, default=700)
    parser.add_argument("--rpm-ramp-time", type=float, default=1.0)
    parser.add_argument("--status-timeout", type=float, default=0.7)
    parser.add_argument("--print-period", type=float, default=0.25)
    parser.add_argument("--drain-rx-frames", type=int, default=256)
    parser.add_argument("--send-timeout", type=float, default=0.01)
    parser.add_argument("--stop-ramp-time", type=float, default=1.0)
    parser.add_argument("--zero-repeats", type=int, default=8)
    parser.add_argument("--zero-gap", type=float, default=0.03)
    parser.add_argument("--allow-high", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    args.reverse_ids = set(args.reverse_ids)
    safety_check(args)
    base_target_erpm = target_erpm(args)
    confirm_if_needed(args, base_target_erpm)

    if args.dry_run:
        print("dry-run: no CAN frames sent")
        return

    bus = open_bus(args)
    start = time.monotonic()
    states = {controller_id: EscState(phase_start=start) for controller_id in args.ids}
    last_print = start
    total_rx = 0

    try:
        print(
            f"Multi kick-start RPM control: ids={','.join(f'0x{x:02X}' for x in args.ids)} "
            f"target={base_target_erpm:+d}ERPM ({base_target_erpm / args.pole_pairs:+.1f}rpm)"
        )
        wait_initial_status(bus, args, states)

        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            total_rx += drain_status(bus, args, states, now)

            for controller_id, state in states.items():
                direction = sign_for(controller_id, args.reverse_ids)
                run_one_esc(bus, args, controller_id, state, direction * abs(base_target_erpm), now)
                time.sleep(args.inter_frame_gap)

            total_rx += drain_status(bus, args, states, time.monotonic())

            if now - last_print >= args.print_period:
                parts = []
                for controller_id, state in states.items():
                    parts.append(
                        f"0x{controller_id:02X}:{state.phase} "
                        f"cmd={state.rpm_command:+6d} erpm={state.latest_erpm:+6d} "
                        f"rpm={state.latest_erpm / args.pole_pairs:+6.0f} "
                        f"cur={state.latest_current:+4.1f}A duty={state.latest_duty:+.3f} fail={state.tx_fail}"
                    )
                print(f"t={now - start:6.2f}s rx={total_rx} " + " | ".join(parts))
                last_print = now

            time.sleep(args.period)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        smooth_stop(bus, args, states)
        summary = " ".join(f"0x{k:02X}:fail={v.tx_fail}" for k, v in states.items())
        print(f"final {summary} rx={total_rx}")
        bus.shutdown()


if __name__ == "__main__":
    main()

