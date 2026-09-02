#!/usr/bin/env python3
"""
Move only the safe RobStride EduLite 05 motors.

Default safe motor IDs:
  0x20, 0x21, 0x22, 0x23

Hard-blocked motor IDs:
  0x28, 0x38

The blocked IDs are never enabled, controlled, or stopped by this script.
If they appear in --ids, the script exits before sending any CAN frames.

Examples:
  python3 robstride_edulite05_four_safe.py --yes
  python3 robstride_edulite05_four_safe.py --mode constant --speed 1.0 --duration 30 --yes
  python3 robstride_edulite05_four_safe.py --mode alternate --speed 1.0 --half-period 0.4 --yes
  python3 robstride_edulite05_four_safe.py --mode roundrobin --slot-period 2.0 --yes
  python3 robstride_edulite05_four_safe.py --auto-recover --fast-detect --yes
  python3 robstride_edulite05_four_safe.py --stop-only --yes
"""

import argparse
import math
import subprocess
import time

import can


COMM_CONTROL = 0x01
COMM_ENABLE = 0x03
COMM_STOP = 0x04
COMM_GET_ID = 0x00

DEFAULT_IDS = (0x20, 0x21, 0x22, 0x23)
FORBIDDEN_IDS = (0x28, 0x38)

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


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_id_list(value: str) -> list[int]:
    return [parse_int(part.strip()) for part in value.split(",") if part.strip()]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def float_to_uint(value: float, lower: float, upper: float) -> int:
    value = clamp(value, lower, upper)
    return int((value - lower) * 65535.0 / (upper - lower))


def make_ext_id(comm_type: int, target_id: int, data2: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_id & 0xFF)


def open_bus(args: argparse.Namespace) -> can.BusABC:
    return can.interface.Bus(
        channel=args.channel,
        interface="socketcan",
        bitrate=args.bitrate,
    )


def send_ext(
    bus: can.BusABC,
    arb_id: int,
    data: bytes,
    dry_run: bool,
    send_timeout: float,
    send_retries: int,
) -> bool:
    if dry_run:
        print(f"dry tx EXT id=0x{arb_id:X} data={' '.join(f'{b:02X}' for b in data)}")
        return True

    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
    for attempt in range(send_retries + 1):
        try:
            bus.send(msg, timeout=send_timeout)
            return True
        except can.CanOperationError as exc:
            if attempt >= send_retries:
                print(f"CAN TX FAILED id=0x{arb_id:X}: {exc}")
                return False
            time.sleep(0.02)
    return False


def drain_rx(bus: can.BusABC, max_frames: int) -> int:
    count = 0
    while count < max_frames:
        msg = bus.recv(timeout=0.0)
        if msg is None:
            break
        count += 1
    return count


def close_bus(bus: can.BusABC) -> None:
    try:
        bus.shutdown()
    except Exception as exc:
        print(f"bus shutdown warning: {exc}")


def reset_socketcan(args: argparse.Namespace) -> bool:
    commands = [
        ["sudo", "ip", "link", "set", args.channel, "down"],
        [
            "sudo",
            "ip",
            "link",
            "set",
            args.channel,
            "type",
            "can",
            "bitrate",
            str(args.bitrate),
            "restart-ms",
            "100",
        ],
        ["sudo", "ip", "link", "set", args.channel, "up"],
    ]

    for command in commands:
        result = subprocess.run(command, text=True, capture_output=True)
        if result.returncode != 0:
            print(f"reset failed: {' '.join(command)}")
            if result.stderr.strip():
                print(result.stderr.strip())
            return False
    return True


def get_id_once(bus: can.BusABC, motor_id: int, args: argparse.Namespace) -> bool:
    arb_id = make_ext_id(COMM_GET_ID, motor_id, args.host_id)
    if not send_ext(bus, arb_id, bytes(8), args.dry_run, args.send_timeout, args.send_retries):
        return False

    deadline = time.monotonic() + args.probe_timeout
    expected_reply_id = ((motor_id & 0xFFFF) << 8) | 0xFE
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.005)
        if msg is None or not msg.is_extended_id:
            continue
        if msg.arbitration_id == expected_reply_id:
            return True
    return False


def probe_safe_ids(bus: can.BusABC, args: argparse.Namespace) -> set[int]:
    found = set()
    for motor_id in args.ids:
        if get_id_once(bus, motor_id, args):
            found.add(motor_id)
        time.sleep(args.inter_frame_gap)
    return found


def wait_for_reconnect(args: argparse.Namespace) -> can.BusABC:
    attempt = 0
    reconnect_started = time.monotonic()
    while True:
        attempt += 1
        print(f"Reconnect attempt {attempt}: resetting {args.channel} and probing safe IDs...")
        reset_socketcan(args)
        time.sleep(args.reconnect_delay)

        bus = open_bus(args)
        found = probe_safe_ids(bus, args)
        missing = [motor_id for motor_id in args.ids if motor_id not in found]
        if not missing:
            elapsed = time.monotonic() - reconnect_started
            print(
                "Reconnect OK after "
                f"{elapsed:.2f}s: "
                + ", ".join(f"0x{x:02X}" for x in sorted(found))
            )
            return bus

        print(
            "Still missing: "
            + ", ".join(f"0x{x:02X}" for x in missing)
            + "; waiting..."
        )
        close_bus(bus)
        time.sleep(args.reconnect_delay)


def enable_motor(bus: can.BusABC, motor_id: int, args: argparse.Namespace) -> bool:
    arb_id = make_ext_id(COMM_ENABLE, motor_id, args.host_id)
    print(f"enable 0x{motor_id:02X}")
    return send_ext(bus, arb_id, bytes(8), args.dry_run, args.send_timeout, args.send_retries)


def stop_motor(bus: can.BusABC, motor_id: int, args: argparse.Namespace) -> bool:
    arb_id = make_ext_id(COMM_STOP, motor_id, args.host_id)
    print(f"stop   0x{motor_id:02X}")
    return send_ext(bus, arb_id, bytes(8), args.dry_run, args.send_timeout, args.send_retries)


def control_data(position: float, velocity: float, kp: float, kd: float) -> bytes:
    data = bytearray(8)
    data[0:2] = float_to_uint(position, P_MIN, P_MAX).to_bytes(2, "big")
    data[2:4] = float_to_uint(velocity, V_MIN, V_MAX).to_bytes(2, "big")
    data[4:6] = float_to_uint(kp, KP_MIN, KP_MAX).to_bytes(2, "big")
    data[6:8] = float_to_uint(kd, KD_MIN, KD_MAX).to_bytes(2, "big")
    return bytes(data)


def op_control(
    bus: can.BusABC,
    motor_id: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    args: argparse.Namespace,
) -> bool:
    torque_raw = float_to_uint(torque, T_MIN, T_MAX)
    arb_id = make_ext_id(COMM_CONTROL, motor_id, torque_raw)
    return send_ext(
        bus,
        arb_id,
        control_data(position, velocity, kp, kd),
        args.dry_run,
        args.send_timeout,
        args.send_retries,
    )


def speed_for(args: argparse.Namespace, motor_index: int, elapsed: float) -> float:
    speed = args.speed
    if args.stagger:
        elapsed += motor_index * args.half_period / max(1, len(args.ids))

    if args.mode == "constant":
        return speed

    phase = int(elapsed / args.half_period) % 2
    return speed if phase == 0 else -speed


def validate_ids(ids: list[int], forbidden: set[int]) -> None:
    if not ids:
        raise SystemExit("No motor IDs selected.")
    bad = sorted(set(ids) & forbidden)
    if bad:
        formatted = ", ".join(f"0x{x:02X}" for x in bad)
        raise SystemExit(f"Refusing to move forbidden motor ID(s): {formatted}")
    if len(ids) != len(set(ids)):
        raise SystemExit("Duplicate motor IDs in --ids.")


def require_confirm(args: argparse.Namespace) -> None:
    if args.yes or args.dry_run:
        return
    ids = ", ".join(f"0x{x:02X}" for x in args.ids)
    forbidden = ", ".join(f"0x{x:02X}" for x in sorted(args.forbidden_ids))
    duration = "until Ctrl+C" if args.duration <= 0 else f"{args.duration}s"
    print(f"Will move only: {ids}")
    print(f"Forbidden IDs are blocked: {forbidden}")
    if args.stop_only:
        print("Mode=stop-only")
    else:
        print(f"Mode={args.mode}, speed={args.speed}rad/s, duration={duration}")
    input("Press Enter to start, or Ctrl+C to cancel. ")


def apply_fast_detect(args: argparse.Namespace) -> None:
    if not args.fast_detect:
        return
    args.period = min(args.period, 0.05)
    args.send_timeout = min(args.send_timeout, 0.005)
    args.send_retries = 0
    args.max_tx_failures = 1
    args.probe_timeout = min(args.probe_timeout, 0.02)
    args.reconnect_delay = min(args.reconnect_delay, 0.1)


def send_zero_and_stop(bus: can.BusABC, motor_ids: list[int], args: argparse.Namespace) -> None:
    for _ in range(5):
        for motor_id in motor_ids:
            op_control(
                bus,
                motor_id,
                position=0.0,
                velocity=0.0,
                kp=0.0,
                kd=args.kd,
                torque=0.0,
                args=args,
            )
            time.sleep(args.inter_frame_gap)
        time.sleep(0.02)

    for motor_id in motor_ids:
        stop_motor(bus, motor_id, args)
        time.sleep(0.03)


def main() -> None:
    parser = argparse.ArgumentParser(description="Move only safe RobStride EduLite 05 motor IDs.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--host-id", type=parse_int, default=0xFD)
    parser.add_argument("--ids", type=parse_id_list, default=list(DEFAULT_IDS), help="Comma-separated IDs, default: 0x20,0x21,0x22,0x23")
    parser.add_argument("--forbidden-ids", type=parse_id_list, default=list(FORBIDDEN_IDS), help="Comma-separated blocked IDs, default: 0x28,0x38")
    parser.add_argument("--mode", choices=("constant", "alternate", "roundrobin"), default="alternate")
    parser.add_argument("--speed", type=float, default=1.0, help="rad/s, default: 1.0")
    parser.add_argument("--half-period", type=float, default=0.4, help="seconds per alternate phase, default: 0.4")
    parser.add_argument("--slot-period", type=float, default=2.0, help="seconds per motor in roundrobin mode, default: 2.0")
    parser.add_argument("--period", type=float, default=0.05, help="command loop period, default: 0.05")
    parser.add_argument("--inter-frame-gap", type=float, default=0.005, help="pause between motor frames, default: 0.005")
    parser.add_argument("--send-timeout", type=float, default=0.02, help="SocketCAN send timeout, default: 0.02")
    parser.add_argument("--send-retries", type=int, default=1, help="send retries after TX failure, default: 1")
    parser.add_argument("--max-tx-failures", type=int, default=3, help="stop after this many consecutive TX failures")
    parser.add_argument("--drain-rx-frames", type=int, default=32, help="discard up to this many received frames per loop")
    parser.add_argument("--auto-recover", action="store_true", help="reset can0, probe safe IDs, and resume after TX failure")
    parser.add_argument("--fast-detect", action="store_true", help="lower TX/probe timeouts for faster disconnect detection")
    parser.add_argument("--probe-timeout", type=float, default=0.05, help="seconds to wait for each get-id reply")
    parser.add_argument("--reconnect-delay", type=float, default=0.5, help="seconds between reconnect attempts")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl+C")
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--stagger", action="store_true", help="Offset phases so motors do not all reverse at once")
    parser.add_argument("--stop-only", action="store_true", help="Send zero velocity and stop to safe IDs, then exit")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    args.forbidden_ids = set(args.forbidden_ids)
    apply_fast_detect(args)
    validate_ids(args.ids, args.forbidden_ids)
    require_confirm(args)

    bus = open_bus(args)
    start = time.monotonic()
    next_print = start
    consecutive_tx_failures = 0
    tx_error_seen = False
    last_roundrobin_index = None

    def enable_all(current_bus: can.BusABC) -> None:
        print("Enabling safe motors only...")
        for safe_id in args.ids:
            if not enable_motor(current_bus, safe_id, args):
                raise RuntimeError(f"Failed to enable motor 0x{safe_id:02X}; check CAN bus/ACK.")
            time.sleep(0.03)

    try:
        if args.stop_only:
            print("Sending zero velocity, then stop, to safe motors only...")
            send_zero_and_stop(bus, args.ids, args)
            return

        enable_all(bus)

        print("Running. Ctrl+C stops only the safe motors.")
        while True:
            now = time.monotonic()
            elapsed = now - start
            if args.duration > 0 and elapsed >= args.duration:
                break

            speeds = []
            if args.mode == "roundrobin":
                active_index = int(elapsed / args.slot_period) % len(args.ids)
                active_id = args.ids[active_index]

                if last_roundrobin_index is not None and active_index != last_roundrobin_index:
                    previous_id = args.ids[last_roundrobin_index]
                    for _ in range(3):
                        op_control(
                            bus,
                            previous_id,
                            position=0.0,
                            velocity=0.0,
                            kp=0.0,
                            kd=args.kd,
                            torque=0.0,
                            args=args,
                        )
                        time.sleep(args.inter_frame_gap)

                last_roundrobin_index = active_index
                velocity = speed_for(args, active_index, elapsed)
                speeds.append(f"0x{active_id:02X}:{velocity:+.1f}")
                ok = op_control(
                    bus,
                    active_id,
                    position=0.0,
                    velocity=velocity,
                    kp=0.0,
                    kd=args.kd,
                    torque=0.0,
                    args=args,
                )
                if not ok:
                    consecutive_tx_failures += 1
                    if consecutive_tx_failures >= args.max_tx_failures:
                        if not args.auto_recover:
                            tx_error_seen = True
                            raise RuntimeError("CAN transmit failed repeatedly; bus may be disconnected or unacknowledged.")
                        print("CAN transmit failed repeatedly; entering reconnect loop.")
                        close_bus(bus)
                        bus = wait_for_reconnect(args)
                        enable_all(bus)
                        consecutive_tx_failures = 0
                        last_roundrobin_index = None
                        break
                else:
                    consecutive_tx_failures = 0
                time.sleep(args.inter_frame_gap)
            else:
                for index, motor_id in enumerate(args.ids):
                    velocity = speed_for(args, index, elapsed)
                    speeds.append(f"0x{motor_id:02X}:{velocity:+.1f}")
                    ok = op_control(
                        bus,
                        motor_id,
                        position=0.0,
                        velocity=velocity,
                        kp=0.0,
                        kd=args.kd,
                        torque=0.0,
                        args=args,
                    )
                    if not ok:
                        consecutive_tx_failures += 1
                        if consecutive_tx_failures >= args.max_tx_failures:
                            if not args.auto_recover:
                                tx_error_seen = True
                                raise RuntimeError("CAN transmit failed repeatedly; bus may be disconnected or unacknowledged.")
                            print("CAN transmit failed repeatedly; entering reconnect loop.")
                            close_bus(bus)
                            bus = wait_for_reconnect(args)
                            enable_all(bus)
                            consecutive_tx_failures = 0
                            last_roundrobin_index = None
                            break
                    else:
                        consecutive_tx_failures = 0
                    time.sleep(args.inter_frame_gap)

            if args.drain_rx_frames > 0:
                drain_rx(bus, args.drain_rx_frames)

            if now >= next_print:
                print("cmd " + " ".join(speeds))
                next_print = now + 0.5

            time.sleep(args.period)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except RuntimeError as exc:
        print(f"\nStopping because of TX error: {exc}")
    finally:
        if tx_error_seen:
            print("TX queue is already full; skipping zero/stop frames.")
            print("Reset can0, then run a stop command after the bus is healthy again.")
        else:
            print("Sending zero velocity, then stop, to safe motors only...")
            send_zero_and_stop(bus, args.ids, args)

        close_bus(bus)


if __name__ == "__main__":
    main()

