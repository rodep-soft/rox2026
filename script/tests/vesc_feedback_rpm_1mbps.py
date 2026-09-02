#!/usr/bin/env python3
"""
Feedback RPM controller for one VESC-compatible ESC on CAN.

This does not rely on VESC SET_RPM closed-loop control. Instead it:
  1. Reads measured ERPM from VESC STATUS frames.
  2. Sends SET_CURRENT commands.
  3. Uses a small PI loop to make measured RPM approach the target.

VESC reports/sends RPM as ERPM in common firmware builds:
  ERPM = mechanical_rpm * pole_pairs

Examples:
  python3 vesc_feedback_rpm_1mbps.py --controller-id 0x32 --target-rpm 100 --duration 10
  python3 vesc_feedback_rpm_1mbps.py --controller-id 0x32 --target-rpm 100 --pole-pairs 21 --max-current 5 --allow-high --yes
  python3 vesc_feedback_rpm_1mbps.py --controller-id 0x32 --target-rpm -100 --pole-pairs 21 --max-current 5 --allow-high --yes
  python3 vesc_feedback_rpm_1mbps.py --controller-id 0x32 --target-rpm 100 --pole-pairs 7 --ramp-down-time 2 --yes
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


def send_rpm_zero(bus, args) -> None:
    can_module = load_can_module()
    arb_id = vesc_id(PACKET_SET_RPM, args.controller_id)
    msg = can_module.Message(
        arbitration_id=arb_id,
        data=(0).to_bytes(4, "big", signed=True),
        is_extended_id=True,
    )
    try:
        bus.send(msg, timeout=args.send_timeout)
    except can_module.CanOperationError as exc:
        print(f"CAN TX FAILED set-rpm-zero id=0x{arb_id:X}: {exc}")


def read_latest_status(bus, args, max_wait: float = 0.0) -> tuple[int | None, float | None, float | None]:
    deadline = time.monotonic() + max_wait
    latest = (None, None, None)

    while True:
        timeout = 0.0 if max_wait <= 0 else max(0.0, min(0.01, deadline - time.monotonic()))
        msg = bus.recv(timeout=timeout)
        if msg is None:
            if max_wait <= 0 or time.monotonic() >= deadline:
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

        if max_wait <= 0:
            return latest


def target_erpm(args) -> int:
    if args.target_erpm is not None:
        return int(args.target_erpm)
    return int(round(args.target_rpm * args.pole_pairs))


def ramped_target_erpm(args, full_target_erpm: int, elapsed: float) -> int:
    ramp_up_scale = 1.0
    if args.ramp_up_time > 0:
        ramp_up_scale = min(1.0, max(0.0, elapsed / args.ramp_up_time))

    ramp_down_scale = 1.0
    if args.duration <= 0 or args.ramp_down_time <= 0:
        return int(round(full_target_erpm * ramp_up_scale))

    ramp_start = max(0.0, args.duration - args.ramp_down_time)
    if elapsed <= ramp_start:
        return int(round(full_target_erpm * ramp_up_scale))

    remaining = max(0.0, args.duration - elapsed)
    ramp_down_scale = remaining / max(args.ramp_down_time, 1e-9)
    scale = min(ramp_up_scale, ramp_down_scale)
    return int(round(full_target_erpm * scale))


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def clamp_current_for_target(value: float, args, effective_erpm_target: int) -> float:
    if effective_erpm_target >= 0:
        lower = -abs(args.max_brake_current)
        upper = args.max_current
    else:
        lower = -args.max_current
        upper = abs(args.max_brake_current)
    return max(lower, min(upper, value))


def slew_limit(value: float, previous: float, rate: float, dt: float) -> float:
    if rate <= 0:
        return value
    max_delta = rate * dt
    return previous + clamp(value - previous, max_delta)


def confirm_if_needed(args, erpm_target: int) -> None:
    if args.yes or args.dry_run:
        return
    print(f"Controller ID: 0x{args.controller_id:02X}")
    print(f"Target: {args.target_rpm:+.1f} mechanical RPM = {erpm_target:+d} ERPM")
    print(f"Current limit: +/-{args.max_current:.1f}A")
    print(f"Duration: {args.duration}s")
    print("Make sure the wheel is unloaded and power can be cut immediately.")
    input("Press Enter to start, or Ctrl+C to cancel. ")


def safety_check(args) -> None:
    if args.pole_pairs <= 0:
        raise SystemExit("--pole-pairs must be positive")
    if args.period <= 0:
        raise SystemExit("--period must be positive")
    if args.ramp_down_time < 0:
        raise SystemExit("--ramp-down-time must be zero or positive")
    if args.ramp_up_time < 0:
        raise SystemExit("--ramp-up-time must be zero or positive")
    if args.stop_ramp_time < 0:
        raise SystemExit("--stop-ramp-time must be zero or positive")
    if not 0 < args.rpm_filter_alpha <= 1:
        raise SystemExit("--rpm-filter-alpha must be > 0 and <= 1")
    if args.current_slew_a_per_s < 0:
        raise SystemExit("--current-slew-a-per-s must be zero or positive")
    if args.max_brake_current < 0:
        raise SystemExit("--max-brake-current must be zero or positive")
    if args.max_current <= 0:
        raise SystemExit("--max-current must be positive")
    if args.status_timeout <= 0:
        raise SystemExit("--status-timeout must be positive")
    if args.allow_high:
        return
    if args.max_current > 3.0:
        raise SystemExit("Refusing --max-current > 3A without --allow-high.")
    if abs(target_erpm(args)) > 4000:
        raise SystemExit("Refusing target above 4000 ERPM without --allow-high.")


def stop_motor(bus, args, last_current_cmd: float) -> None:
    if args.stop_ramp_time > 0 and abs(last_current_cmd) > 0.05:
        steps = max(1, int(args.stop_ramp_time / max(args.stop_gap, 1e-3)))
        print(f"Ramping current to zero over {args.stop_ramp_time:.2f}s...")
        for step in range(steps - 1, -1, -1):
            scale = step / steps
            send_current(bus, args, last_current_cmd * scale)
            time.sleep(args.stop_gap)

    print("Sending zero current/rpm...")
    for _ in range(args.stop_repeats):
        send_current(bus, args, 0.0)
        time.sleep(args.stop_gap)
    send_rpm_zero(bus, args)


def main() -> None:
    parser = argparse.ArgumentParser(description="Outer-loop RPM control for one VESC using CAN SET_CURRENT.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--controller-id", type=parse_int, default=0x32)
    parser.add_argument("--target-rpm", type=float, default=100.0, help="mechanical RPM target")
    parser.add_argument("--target-erpm", type=int, default=None, help="override target as VESC ERPM")
    parser.add_argument("--pole-pairs", type=int, default=21)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=0.02)
    parser.add_argument("--status-timeout", type=float, default=0.5)
    parser.add_argument("--ramp-up-time", type=float, default=1.5, help="seconds to ramp target RPM up from zero")
    parser.add_argument("--ramp-down-time", type=float, default=1.0, help="seconds to ramp target RPM to zero before normal finish")
    parser.add_argument("--max-current", type=float, default=3.0)
    parser.add_argument("--max-brake-current", type=float, default=0.0, help="reverse/braking current limit; default disables active braking")
    parser.add_argument("--current-slew-a-per-s", type=float, default=8.0, help="limit current-command change rate; 0 disables")
    parser.add_argument("--startup-current", type=float, default=2.0)
    parser.add_argument("--startup-time", type=float, default=0.35)
    parser.add_argument("--feedforward-current", type=float, default=0.3)
    parser.add_argument("--kp", type=float, default=0.00035, help="A per ERPM error")
    parser.add_argument("--ki", type=float, default=0.00008, help="A per ERPM-second error")
    parser.add_argument("--rpm-filter-alpha", type=float, default=0.25, help="low-pass filter coefficient for measured ERPM")
    parser.add_argument("--deadband-erpm", type=float, default=80.0)
    parser.add_argument("--print-period", type=float, default=0.2)
    parser.add_argument("--send-timeout", type=float, default=0.01)
    parser.add_argument("--stop-ramp-time", type=float, default=0.6, help="seconds to ramp last current command to zero on stop")
    parser.add_argument("--stop-repeats", type=int, default=8)
    parser.add_argument("--stop-gap", type=float, default=0.03)
    parser.add_argument("--allow-high", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    safety_check(args)
    erpm_target = target_erpm(args)
    target_sign = 1.0 if erpm_target >= 0 else -1.0
    confirm_if_needed(args, erpm_target)

    if args.dry_run:
        print("dry-run: no CAN frames sent")
        return

    bus = open_bus(args)
    start = time.monotonic()
    last_time = start
    last_print = start
    last_status_time = None
    latest_erpm = None
    filtered_erpm = None
    latest_motor_current = None
    latest_duty = None
    integral = 0.0
    tx_fail = 0
    last_current_cmd = 0.0

    try:
        print(
            f"Running feedback RPM control: id=0x{args.controller_id:02X} "
            f"target={args.target_rpm:+.1f}rpm target_erpm={erpm_target:+d}"
        )

        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            dt = max(1e-3, now - last_time)
            last_time = now

            erpm, motor_current, duty = read_latest_status(bus, args, max_wait=0.0)
            if erpm is not None:
                latest_erpm = erpm
                if filtered_erpm is None:
                    filtered_erpm = float(erpm)
                else:
                    filtered_erpm += args.rpm_filter_alpha * (float(erpm) - filtered_erpm)
                latest_motor_current = motor_current
                latest_duty = duty
                last_status_time = now

            if last_status_time is None:
                erpm, motor_current, duty = read_latest_status(bus, args, max_wait=min(args.status_timeout, 0.2))
                if erpm is not None:
                    latest_erpm = erpm
                    if filtered_erpm is None:
                        filtered_erpm = float(erpm)
                    else:
                        filtered_erpm += args.rpm_filter_alpha * (float(erpm) - filtered_erpm)
                    latest_motor_current = motor_current
                    latest_duty = duty
                    last_status_time = time.monotonic()
                    now = last_status_time

            if last_status_time is None or now - last_status_time > args.status_timeout:
                print("Feedback lost: no STATUS frames from target ESC. Stopping.")
                break

            elapsed = now - start
            effective_erpm_target = ramped_target_erpm(args, erpm_target, elapsed)
            effective_target_sign = 1.0 if effective_erpm_target >= 0 else -1.0
            feedback_erpm = int(round(filtered_erpm if filtered_erpm is not None else latest_erpm))
            error = effective_erpm_target - feedback_erpm

            if (
                elapsed < args.startup_time
                and abs(effective_erpm_target) > args.deadband_erpm
                and abs(feedback_erpm) < abs(effective_erpm_target) * 0.4
            ):
                current_cmd = effective_target_sign * abs(args.startup_current)
                phase = "startup"
                integral = 0.0
            else:
                phase = "rampdown" if abs(effective_erpm_target) < abs(erpm_target) else "feedback"
                if abs(error) <= args.deadband_erpm:
                    error = 0
                integral += error * dt
                integral = clamp(integral, args.max_current / max(args.ki, 1e-9))
                current_cmd = args.kp * error + args.ki * integral
                if error != 0:
                    current_cmd += effective_target_sign * abs(args.feedforward_current)
                current_cmd = clamp_current_for_target(current_cmd, args, effective_erpm_target)

            current_cmd = slew_limit(current_cmd, last_current_cmd, args.current_slew_a_per_s, dt)
            current_cmd = clamp_current_for_target(current_cmd, args, effective_erpm_target)

            if not send_current(bus, args, current_cmd):
                tx_fail += 1
                if tx_fail >= 3:
                    print("Stopping after repeated CAN TX failures.")
                    break
            else:
                last_current_cmd = current_cmd

            if now - last_print >= args.print_period:
                measured_rpm = feedback_erpm / args.pole_pairs
                raw_rpm = latest_erpm / args.pole_pairs
                effective_target_rpm = effective_erpm_target / args.pole_pairs
                print(
                    f"t={elapsed:6.2f}s {phase:<8} "
                    f"target={effective_target_rpm:+7.1f}rpm measured={measured_rpm:+7.1f}rpm raw={raw_rpm:+7.1f}rpm "
                    f"erpm={latest_erpm:+7d} err={error:+7.0f} "
                    f"cmd={current_cmd:+5.2f}A "
                    f"vesc_current={latest_motor_current:+5.1f}A duty={latest_duty:+.3f} "
                    f"tx_fail={tx_fail}"
                )
                last_print = now

            sleep_time = args.period - (time.monotonic() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        stop_motor(bus, args, last_current_cmd)
        bus.shutdown()


if __name__ == "__main__":
    main()

