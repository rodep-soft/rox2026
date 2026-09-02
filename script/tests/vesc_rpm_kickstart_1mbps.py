#!/usr/bin/env python3
"""
Kick-start one VESC-compatible ESC with current, then hand off to SET_RPM.

This is useful when plain SET_RPM makes the motor buzz/try to move but cannot
start under belt/flywheel load. The flow is:
  1. Send SET_CURRENT for a short spin-up window.
  2. Watch STATUS rpm.
  3. Once above --handoff-erpm, switch to VESC SET_RPM.
  4. If speed drops below --rekick-erpm, kick again.

Examples:
  python3 vesc_rpm_kickstart_1mbps.py --controller-id 0x33 --target-rpm 5000 --duration 10 --yes
  python3 vesc_rpm_kickstart_1mbps.py --controller-id 0x33 --target-erpm 5000 --duration 10 --yes
  python3 vesc_rpm_kickstart_1mbps.py --controller-id 0x33 --target-erpm 5000 --kick-current 8 --kick-time 1.0 --yes
"""

from __future__ import annotations

import argparse
import struct
import time


PACKET_SET_CURRENT = 1
PACKET_SET_RPM = 3
PACKET_STATUS = 9

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


def be_i16(data: bytes, offset: int) -> int:
    return struct.unpack(">h", data[offset : offset + 2])[0]


def be_i32(data: bytes, offset: int) -> int:
    return struct.unpack(">i", data[offset : offset + 4])[0]


def open_bus(args):
    can_module = load_can_module()
    return can_module.interface.Bus(channel=args.channel, interface="socketcan", bitrate=args.bitrate)


def send_current(bus, args, current_a: float) -> bool:
    can_module = load_can_module()
    raw = int(current_a * 1000.0)
    arb_id = vesc_id(PACKET_SET_CURRENT, args.controller_id)
    msg = can_module.Message(
        arbitration_id=arb_id,
        data=raw.to_bytes(4, "big", signed=True),
        is_extended_id=True,
    )
    try:
        bus.send(msg, timeout=args.send_timeout)
        return True
    except can_module.CanOperationError as exc:
        print(f"CAN TX FAILED set-current id=0x{arb_id:X}: {exc}")
        return False


def send_rpm(bus, args, erpm: int) -> bool:
    can_module = load_can_module()
    arb_id = vesc_id(PACKET_SET_RPM, args.controller_id)
    msg = can_module.Message(
        arbitration_id=arb_id,
        data=int(erpm).to_bytes(4, "big", signed=True),
        is_extended_id=True,
    )
    try:
        bus.send(msg, timeout=args.send_timeout)
        return True
    except can_module.CanOperationError as exc:
        print(f"CAN TX FAILED set-rpm id=0x{arb_id:X}: {exc}")
        return False


def read_status(bus, args, wait: float = 0.0) -> tuple[int | None, float | None, float | None]:
    deadline = time.monotonic() + wait
    latest = (None, None, None)

    while True:
        timeout = 0.0 if wait <= 0 else max(0.0, min(0.01, deadline - time.monotonic()))
        msg = bus.recv(timeout=timeout)
        if msg is None:
            if wait <= 0 or time.monotonic() >= deadline:
                return latest
            continue
        if not msg.is_extended_id:
            continue
        packet = (msg.arbitration_id >> 8) & 0xFF
        controller_id = msg.arbitration_id & 0xFF
        if packet != PACKET_STATUS or controller_id != args.controller_id:
            continue
        data = bytes(msg.data)
        if len(data) < 8:
            continue
        erpm = be_i32(data, 0)
        motor_current = be_i16(data, 4) / 10.0
        duty = be_i16(data, 6) / 1000.0
        latest = (erpm, motor_current, duty)
        if wait <= 0:
            return latest


def target_erpm(args) -> int:
    if args.target_erpm is not None:
        return args.target_erpm
    return int(round(args.target_rpm * args.pole_pairs))


def sign(value: int | float) -> int:
    return 1 if value >= 0 else -1


def ramped_value(target: float, elapsed: float, ramp_time: float) -> float:
    if ramp_time <= 0:
        return target
    return target * min(1.0, max(0.0, elapsed / ramp_time))


def ramp_between(start_value: float, target_value: float, elapsed: float, ramp_time: float) -> float:
    if ramp_time <= 0:
        return target_value
    scale = min(1.0, max(0.0, elapsed / ramp_time))
    return start_value + (target_value - start_value) * scale


def confirm_if_needed(args, erpm_target: int) -> None:
    if args.yes or args.dry_run:
        return
    print(f"Controller ID: 0x{args.controller_id:02X}")
    print(f"Target: {erpm_target:+d} ERPM ({erpm_target / args.pole_pairs:+.1f} mechanical RPM if pole_pairs={args.pole_pairs})")
    print(f"Kick current: {args.kick_current:+.1f}A for up to {args.kick_time}s")
    print(f"Handoff threshold: {args.handoff_erpm} ERPM")
    print("Make sure the belt/flywheel is guarded and power can be cut immediately.")
    input("Press Enter to start, or Ctrl+C to cancel. ")


def safety_check(args) -> None:
    if args.pole_pairs <= 0:
        raise SystemExit("--pole-pairs must be positive")
    if args.period <= 0:
        raise SystemExit("--period must be positive")
    if args.handoff_erpm < 0 or args.rekick_erpm < 0:
        raise SystemExit("--handoff-erpm and --rekick-erpm must be zero or positive")
    if args.stop_ramp_time < 0 or args.rpm_ramp_time < 0:
        raise SystemExit("ramp times must be zero or positive")
    if args.allow_high:
        return
    if abs(args.kick_current) > 3.0:
        raise SystemExit("Refusing --kick-current > 3A without --allow-high.")
    if abs(target_erpm(args)) > 4000:
        raise SystemExit("Refusing target above 4000 ERPM without --allow-high.")


def smooth_stop(bus, args, current_erpm: int) -> None:
    target = current_erpm
    if args.stop_ramp_time > 0:
        steps = max(1, int(args.stop_ramp_time / max(args.period, 1e-3)))
        print(f"Ramping SET_RPM to zero over {args.stop_ramp_time:.2f}s...")
        for step in range(steps - 1, -1, -1):
            send_rpm(bus, args, int(target * step / steps))
            time.sleep(args.period)

    print("Sending zero current/rpm...")
    for _ in range(args.zero_repeats):
        send_current(bus, args, 0.0)
        time.sleep(args.zero_gap)
    send_rpm(bus, args, 0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Kick-start a VESC with current, then run internal SET_RPM.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--controller-id", type=parse_int, default=0x33)
    parser.add_argument("--target-rpm", type=float, default=1000.0, help="mechanical RPM target")
    parser.add_argument("--target-erpm", type=int, default=None, help="override target as VESC ERPM")
    parser.add_argument("--pole-pairs", type=int, default=7)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--kick-current", type=float, default=6.0)
    parser.add_argument("--kick-time", type=float, default=0.8)
    parser.add_argument("--handoff-erpm", type=int, default=900)
    parser.add_argument("--rekick-erpm", type=int, default=500)
    parser.add_argument("--rpm-ramp-time", type=float, default=1.0, help="ramp SET_RPM target after handoff")
    parser.add_argument("--status-timeout", type=float, default=0.7)
    parser.add_argument("--print-period", type=float, default=0.2)
    parser.add_argument("--send-timeout", type=float, default=0.01)
    parser.add_argument("--stop-ramp-time", type=float, default=1.0)
    parser.add_argument("--zero-repeats", type=int, default=8)
    parser.add_argument("--zero-gap", type=float, default=0.03)
    parser.add_argument("--allow-high", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    safety_check(args)
    erpm_target = target_erpm(args)
    direction = sign(erpm_target)
    kick_current = direction * abs(args.kick_current)
    handoff = abs(args.handoff_erpm)
    rekick = abs(args.rekick_erpm)
    confirm_if_needed(args, erpm_target)

    if args.dry_run:
        print("dry-run: no CAN frames sent")
        return

    bus = open_bus(args)
    start = time.monotonic()
    phase_start = start
    last_status_time = None
    last_print = start
    latest_erpm = 0
    latest_current = 0.0
    latest_duty = 0.0
    phase = "kick"
    tx_fail = 0
    rpm_command = 0
    rpm_ramp_start_erpm = 0

    try:
        print(
            f"Kick-start RPM control: id=0x{args.controller_id:02X} "
            f"target={erpm_target:+d}ERPM ({erpm_target / args.pole_pairs:+.1f}rpm)"
        )

        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            erpm, motor_current, duty = read_status(bus, args, wait=0.0)
            if erpm is not None:
                latest_erpm = erpm
                latest_current = motor_current
                latest_duty = duty
                last_status_time = now

            if last_status_time is None:
                erpm, motor_current, duty = read_status(bus, args, wait=min(args.status_timeout, 0.2))
                if erpm is not None:
                    latest_erpm = erpm
                    latest_current = motor_current
                    latest_duty = duty
                    last_status_time = time.monotonic()
                    now = last_status_time

            if last_status_time is None or now - last_status_time > args.status_timeout:
                print("Feedback lost: no STATUS frames. Stopping.")
                break

            speed_abs = abs(latest_erpm)
            phase_elapsed = now - phase_start

            if phase == "kick":
                ok = send_current(bus, args, kick_current)
                if speed_abs >= handoff or phase_elapsed >= args.kick_time:
                    phase = "rpm"
                    phase_start = now
                    ramp_start_abs = min(max(speed_abs, handoff), abs(erpm_target))
                    rpm_ramp_start_erpm = int(direction * ramp_start_abs)
                    rpm_command = rpm_ramp_start_erpm
            else:
                if speed_abs < rekick:
                    print(f"Speed fell below rekick threshold: erpm={latest_erpm:+d}. Kicking again.")
                    phase = "kick"
                    phase_start = now
                    ok = send_current(bus, args, kick_current)
                else:
                    ramped = ramp_between(rpm_ramp_start_erpm, erpm_target, phase_elapsed, args.rpm_ramp_time)
                    rpm_command = int(ramped)
                    ok = send_rpm(bus, args, rpm_command)

            if not ok:
                tx_fail += 1
                if tx_fail >= 3:
                    print("Stopping after repeated CAN TX failures.")
                    break

            if now - last_print >= args.print_period:
                print(
                    f"t={now - start:6.2f}s {phase:<4} "
                    f"cmd_erpm={rpm_command:+7d} measured_erpm={latest_erpm:+7d} "
                    f"measured_rpm={latest_erpm / args.pole_pairs:+7.1f} "
                    f"current={latest_current:+5.1f}A duty={latest_duty:+.3f} tx_fail={tx_fail}"
                )
                last_print = now

            time.sleep(args.period)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        smooth_stop(bus, args, rpm_command)
        bus.shutdown()


if __name__ == "__main__":
    main()

