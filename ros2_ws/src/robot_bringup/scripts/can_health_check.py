#!/usr/bin/env python3
"""
CAN & System Health Checker for ROX2026 Robot
CANバス統計、データレート、ノード別フレームレート(Hz)、トポロジー状態の自動診断
"""

import subprocess
import time
import threading
import re
from collections import defaultdict

NODE_MAP = {
    0x020: "Front Left Wheel (EduLite)",
    0x021: "Front Right Wheel (EduLite)",
    0x022: "Rear Left Wheel (EduLite)",
    0x023: "Rear Right Wheel (EduLite)",
    0x028: "Spring / Kicker (EduLite)",
    0x038: "Dribble Motor (EduLite)",
    0x100: "VESC Group Broadcast / Feedback",
    0x200: "STM32 Status / RX",
    0x201: "STM32 Control / TX",
    0x310: "Limit Switch / Sensor Event",
    0x320: "Upper Belt Motor (VESC)",
    0x321: "Under Belt Motor (VESC)",
    0x322: "Dribble Roller Motor (VESC)",
}


def print_header(title):
    print(f"\n--- {title} ---")


def check_can_interface_stats():
    print_header("1. SocketCAN (can0) Status & Error Counters")
    try:
        res = subprocess.run(
            ["ip", "-details", "-s", "link", "show", "can0"], capture_output=True, text=True
        )
        output = res.stdout
        
        state_match = re.search(r"state\s+([A-Z\-]+)", output)
        bus_state = state_match.group(1) if state_match else "UNKNOWN"
        
        if bus_state == "UP":
            print("  State          : \033[92mUP\033[0m")
        elif "BUS-OFF" in bus_state or "BUS-OFF" in output:
            print("  State          : \033[91mBUS-OFF\033[0m")
        else:
            print(f"  State          : \033[93m{bus_state}\033[0m")

        bitrate_match = re.search(r"bitrate\s+(\d+)", output)
        bitrate = bitrate_match.group(1) if bitrate_match else "N/A"
        restart_match = re.search(r"restart-ms\s+(\d+)", output)
        restart_ms = restart_match.group(1) if restart_match else "0"
        print(f"  Bitrate        : {int(bitrate)//1000 if bitrate.isdigit() else bitrate} kbps (restart-ms: {restart_ms}ms)")

        tec_rec_match = re.search(r"berr-counter\s+tec\s+(\d+)\s+rec\s+(\d+)", output)
        if tec_rec_match:
            tec, rec = int(tec_rec_match.group(1)), int(tec_rec_match.group(2))
            tec_str = f"\033[91m{tec}\033[0m" if tec > 96 else f"{tec}"
            rec_str = f"\033[91m{rec}\033[0m" if rec > 96 else f"{rec}"
            print(f"  Error Counters : TEC={tec_str}, REC={rec_str}")

        restarts_match = re.search(r"restarts\s+(\d+)", output)
        bus_errors_match = re.search(r"bus-errors\s+(\d+)", output)
        if restarts_match or bus_errors_match:
            restarts = restarts_match.group(1) if restarts_match else "0"
            bus_errs = bus_errors_match.group(1) if bus_errors_match else "0"
            print(f"  Bus Statistics : Restarts={restarts}, BusErrors={bus_errs}")

    except Exception as e:
        print(f"  [ERROR] Failed to inspect can0 stats: {e}")


def send_pings():
    time.sleep(0.3)
    for can_id in NODE_MAP.keys():
        try:
            cmd = f"cansend can0 {can_id:03X}#0000"
            subprocess.run(cmd, shell=True, capture_output=True)
            time.sleep(0.02)
        except Exception:
            pass


def analyze_can_bus_traffic():
    print_header("2. CAN Node Traffic Analysis (2.0s sample)")
    node_counts = defaultdict(int)
    unknown_nodes = defaultdict(int)
    total_frames = 0

    ping_thread = threading.Thread(target=send_pings)
    ping_thread.daemon = True

    try:
        proc = subprocess.Popen(
            ["candump", "-L", "can0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        ping_thread.start()

        start_time = time.time()
        sample_duration = 2.0
        
        while time.time() - start_time < sample_duration:
            line = proc.stdout.readline()
            if not line:
                break
            
            parts = line.strip().split()
            if len(parts) >= 3:
                try:
                    can_data_str = parts[2]
                    id_data = can_data_str.split('#')
                    can_id = int(id_data[0], 16)
                    data_hex = id_data[1] if len(id_data) > 1 else ""

                    if data_hex == "0000" and can_id in NODE_MAP:
                        continue

                    total_frames += 1
                    if can_id in NODE_MAP:
                        node_counts[can_id] += 1
                    else:
                        unknown_nodes[can_id] += 1
                except (ValueError, IndexError):
                    pass

        proc.terminate()
        proc.wait(timeout=1.0)

    except FileNotFoundError:
        print("  [ERROR] candump command not found.")
        return
    except Exception as e:
        print(f"  [ERROR] Capture exception: {e}")
        return

    fps = total_frames / sample_duration
    bus_load_pct = (fps * 111.0 / 1000000.0) * 100.0

    print(f"  Total Traffic  : {total_frames} frames ({fps:.1f} fps)")
    print(f"  Est. Bus Load  : {bus_load_pct:.2f} %")
    print("\n  Node Status:")

    for can_id, name in NODE_MAP.items():
        count = node_counts[can_id]
        node_fps = count / sample_duration
        if count > 0:
            status_str = f"\033[92mONLINE ({node_fps:5.1f} Hz)\033[0m"
        else:
            status_str = f"\033[91mNO RESPONSE\033[0m"
        
        print(f"  [0x{can_id:03X}] {name:<32} : {status_str}")

    if unknown_nodes:
        print("\n  Unmapped IDs:")
        for u_id, count in unknown_nodes.items():
            u_fps = count / sample_duration
            print(f"  [0x{u_id:03X}] Unknown Node                    : {u_fps:.1f} Hz")

    print_header("3. Diagnostics Insight")
    edulite_online = sum(1 for cid in [0x020, 0x021, 0x022, 0x023, 0x028, 0x038] if node_counts[cid] > 0)
    vesc_online = sum(1 for cid in [0x320, 0x321, 0x322] if node_counts[cid] > 0 or node_counts[0x100] > 0)
    stm32_online = node_counts[0x200] > 0 or node_counts[0x201] > 0

    if edulite_online == 0 and stm32_online and vesc_online:
        print("  Insight: EduLite nodes (0x20~0x38) non-responsive. Check EduLite 24V/12V power or CAN cable.")
    elif edulite_online == 0 and not stm32_online and vesc_online:
        print("  Insight: Only VESC online. Check CAN line between VESC and STM32/EduLite, or main logic power.")
    elif edulite_online > 0 and edulite_online < 6:
        offline_nodes = [name for cid, name in NODE_MAP.items() if cid < 0x100 and node_counts[cid] == 0]
        print(f"  Insight: Partial EduLite offline ({', '.join(offline_nodes)}). Check specific CAN connectors.")
    elif edulite_online == 6 and vesc_online and stm32_online:
        print("  Insight: All nodes operational.")
    else:
        print("  Insight: Partial communication active. Check termination resistors (120 Ohm) or signal noise.")


def check_joystick():
    print_header("4. Joystick Status")
    try:
        res = subprocess.run(["ls", "-l", "/dev/input/js0"], capture_output=True, text=True)
        if res.returncode == 0:
            print("  Joystick (/dev/input/js0): \033[92mCONNECTED\033[0m")
        else:
            print("  Joystick (/dev/input/js0): \033[91mNOT CONNECTED\033[0m")
    except Exception as e:
        print(f"  [ERROR] {e}")


if __name__ == "__main__":
    check_can_interface_stats()
    analyze_can_bus_traffic()
    check_joystick()
    print()


