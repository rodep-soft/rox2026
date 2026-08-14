#!/usr/bin/env python3
"""
CAN & System Health Checker for ROX2026 Robot
CANバス統計、データレート、ノード別フレームレート(Hz)、トポロジー状態の自動診断

[修正] EduLite Extended CAN (29-bit ID) フレームに対応
  ID構造: (TYPE << 24) | (STATUS << 16) | (MOTOR_ID << 8) | HOST_ID(0xFD)
  TYPE_FEEDBACK=0x02, TYPE_READ_RESP=0x11
"""

import subprocess
import time
import re
from collections import defaultdict

# ─── EduLite: Extended CAN (29-bit ID) ───────────────────────────────────────
EDULITE_HOST_ID       = 0xFD
EDULITE_TYPE_FEEDBACK = 0x02   # フィードバック
EDULITE_TYPE_READ     = 0x11   # パラメータ読み出し応答
EDULITE_MOTOR_MAP = {
    0x20: "Front Left Wheel  (EduLite)",
    0x21: "Front Right Wheel (EduLite)",
    0x22: "Rear Left Wheel   (EduLite)",
    0x23: "Rear Right Wheel  (EduLite)",
    0x28: "Spring / Kicker   (EduLite)",
    0x38: "Dribble Motor     (EduLite)",
}

# ─── Standard CAN (11-bit ID) ─────────────────────────────────────────────────
STANDARD_NODE_MAP = {
    0x100: "VESC Group Broadcast / Feedback",
    0x200: "STM32 Status / RX              ",
    0x201: "STM32 Control / TX             ",
    0x310: "Limit Switch / Sensor Event    ",
    0x320: "Upper Belt Motor (VESC)        ",
    0x321: "Under Belt Motor (VESC)        ",
    0x322: "Dribble Roller Motor (VESC)    ",
}

RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"


def print_header(title):
    print(f"\n--- {title} ---")


def check_can_interface_stats():
    print_header("1. SocketCAN (can0) Status & Error Counters")
    try:
        res = subprocess.run(
            ["ip", "-details", "-s", "link", "show", "can0"],
            capture_output=True, text=True
        )
        output = res.stdout

        # CAN バス状態（"can state ..." の行を優先して解析）
        can_state_match = re.search(r"can state\s+([A-Z_\-]+)", output)
        link_state_match = re.search(r"state\s+([A-Z]+)", output)
        bus_state = (can_state_match.group(1) if can_state_match
                     else (link_state_match.group(1) if link_state_match else "UNKNOWN"))

        if "ERROR_PASSIVE" in bus_state or "ERROR-PASSIVE" in bus_state:
            print(f"  State          : {RED}{BOLD}ERROR-PASSIVE  ← CAN バスにエラー多発中!{RESET}")
        elif "BUS_OFF" in bus_state or "BUS-OFF" in bus_state:
            print(f"  State          : {RED}{BOLD}BUS-OFF  ← 通信不可能状態!{RESET}")
        elif "UP" in bus_state:
            print(f"  State          : {GREEN}UP (Error Active){RESET}")
        else:
            print(f"  State          : {YELLOW}{bus_state}{RESET}")

        bitrate_match  = re.search(r"bitrate\s+(\d+)", output)
        restart_match  = re.search(r"restart-ms\s+(\d+)", output)
        bitrate    = bitrate_match.group(1)  if bitrate_match  else "N/A"
        restart_ms = restart_match.group(1)  if restart_match  else "0"
        print(f"  Bitrate        : {int(bitrate)//1000 if bitrate.isdigit() else bitrate} kbps"
              f"  (restart-ms: {restart_ms}ms)")

        # TEC / REC
        berr_match = re.search(r"berr-counter\s+tx\s+(\d+)\s+rx\s+(\d+)", output)
        if berr_match:
            tec, rec = int(berr_match.group(1)), int(berr_match.group(2))
            def _color(v):
                if v > 127: return f"{RED}{BOLD}{v}{RESET}"
                if v > 96:  return f"{RED}{v}{RESET}"
                if v > 0:   return f"{YELLOW}{v}{RESET}"
                return str(v)
            print(f"  Error Counters : TEC={_color(tec)}, REC={_color(rec)}")
            if tec > 96 or rec > 96:
                print(f"  {RED}⚠ エラーカウンタ高値: CAN フレームに ACK が返っていないか、"
                      f"エラーフレームが送出されています{RESET}")

        restarts_match   = re.search(r"restarts\s+(\d+)", output)
        bus_errors_match = re.search(r"bus-errors\s+(\d+)", output)
        if restarts_match or bus_errors_match:
            restarts  = restarts_match.group(1)   if restarts_match   else "0"
            bus_errs  = bus_errors_match.group(1) if bus_errors_match else "0"
            print(f"  Bus Statistics : Restarts={restarts}, BusErrors={bus_errs}")

    except Exception as e:
        print(f"  [ERROR] インターフェース情報の取得に失敗: {e}")


def analyze_can_bus_traffic():
    print_header("2. CAN Node Traffic Analysis (2.0s sample)")

    std_counts     = defaultdict(int)
    edulite_counts = defaultdict(int)
    unknown_ext    = defaultdict(int)
    unknown_std    = defaultdict(int)
    error_frames   = 0
    total_frames   = 0
    sample_duration = 2.0

    try:
        proc = subprocess.Popen(
            ["candump", "-L", "can0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        start_time = time.time()
        while time.time() - start_time < sample_duration:
            line = proc.stdout.readline()
            if not line:
                break

            # エラーフレーム
            if "ERRORFRAME" in line:
                error_frames += 1
                continue

            parts = line.strip().split()
            if len(parts) < 3:
                continue
            try:
                id_str, data_str = parts[2].split("#", 1)
                can_id = int(id_str, 16)
                total_frames += 1

                if can_id > 0x7FF:
                    # ─── Extended CAN (EduLite) ───────────────────────────
                    frame_type = (can_id >> 24) & 0x1F
                    motor_id   = (can_id >> 8)  & 0xFF
                    host_id    = can_id          & 0xFF
                    if (host_id == EDULITE_HOST_ID
                            and frame_type in (EDULITE_TYPE_FEEDBACK, EDULITE_TYPE_READ)
                            and motor_id in EDULITE_MOTOR_MAP):
                        edulite_counts[motor_id] += 1
                    else:
                        unknown_ext[can_id] += 1
                else:
                    # ─── Standard CAN ────────────────────────────────────
                    if can_id in STANDARD_NODE_MAP:
                        std_counts[can_id] += 1
                    else:
                        unknown_std[can_id] += 1

            except (ValueError, IndexError):
                pass

        proc.terminate()
        proc.wait(timeout=1.0)

    except FileNotFoundError:
        print("  [ERROR] candump が見つかりません。")
        return
    except Exception as e:
        print(f"  [ERROR] キャプチャ中に例外: {e}")
        return

    fps           = total_frames / sample_duration
    bus_load_pct  = (fps * 111.0 / 1_000_000.0) * 100.0

    print(f"  Total Traffic  : {total_frames} frames ({fps:.1f} fps)")
    print(f"  Est. Bus Load  : {bus_load_pct:.2f} %")
    if error_frames > 0:
        print(f"  {RED}Error Frames   : {error_frames}  ← CAN バス上でエラーフレームを検出!{RESET}")

    # ── EduLite (Extended 29-bit) ──
    print(f"\n  {CYAN}[EduLite  Extended CAN 29-bit]{RESET}")
    for motor_id, name in EDULITE_MOTOR_MAP.items():
        count    = edulite_counts[motor_id]
        node_fps = count / sample_duration
        status   = (f"{GREEN}ONLINE ({node_fps:5.1f} Hz){RESET}"
                    if count > 0 else f"{RED}NO RESPONSE{RESET}")
        print(f"  [0x{motor_id:03X}] {name:<34} : {status}")

    # ── Standard CAN (11-bit) ──
    print(f"\n  {CYAN}[Standard CAN  11-bit]{RESET}")
    for can_id, name in STANDARD_NODE_MAP.items():
        count    = std_counts[can_id]
        node_fps = count / sample_duration
        status   = (f"{GREEN}ONLINE ({node_fps:5.1f} Hz){RESET}"
                    if count > 0 else f"{RED}NO RESPONSE{RESET}")
        print(f"  [0x{can_id:03X}] {name:<34} : {status}")

    # ── 未登録ノード ──
    all_unknown = [(f"EXT 0x{k:08X}", v) for k, v in unknown_ext.items()] + \
                  [(f"STD 0x{k:03X}",  v) for k, v in unknown_std.items()]
    if all_unknown:
        print(f"\n  {YELLOW}[未登録ノード]{RESET}")
        for label, count in sorted(all_unknown, key=lambda x: x[0]):
            print(f"  [{label}]  {count/sample_duration:.1f} Hz")

    # ── Diagnostics Insight ──
    print_header("3. Diagnostics Insight")

    edulite_online = sum(1 for mid in EDULITE_MOTOR_MAP if edulite_counts[mid] > 0)
    vesc_online    = any(std_counts[cid] > 0 for cid in [0x100, 0x320, 0x321, 0x322])
    stm32_online   = std_counts[0x200] > 0 or std_counts[0x310] > 0

    if error_frames > 5:
        print(f"  {RED}⚠ CAN バスにエラーフレームが多発 ({error_frames}件){RESET}")
        print(f"    → EduLite が ACK を返さない / フォーマット不一致 / 配線問題の可能性")

    if edulite_online == 0 and vesc_online:
        print("  ⚠ EduLite ノード全滅。")
        print("    → EduLite の電源 (24V/12V) を確認。")
        print("    → Extended フレーム(29-bit)として送信できているか `candump can0` で確認。")
    elif edulite_online > 0 and edulite_online < len(EDULITE_MOTOR_MAP):
        offline = [name.strip() for mid, name in EDULITE_MOTOR_MAP.items()
                   if edulite_counts[mid] == 0]
        print(f"  ⚠ EduLite 一部応答なし: {', '.join(offline)}")
        print("    → 該当ノードの CAN コネクタ・電源を確認。")
    elif edulite_online == len(EDULITE_MOTOR_MAP) and vesc_online and stm32_online:
        print(f"  {GREEN}✓ 全ノード正常動作中{RESET}")
    else:
        print("  Insight: 部分的な通信のみ。詳細は上のノード状態を確認してください。")

    if not vesc_online:
        print("  ⚠ VESC 応答なし。CAN 配線を確認。")
    if not stm32_online:
        print("  ⚠ STM32 応答なし。STM32 の電源・ファームウェアを確認。")

    return {
        "edulite_online": edulite_online,
        "edulite_counts": dict(edulite_counts),
        "std_counts": dict(std_counts),
        "error_frames": error_frames,
    }


def check_joystick():
    print_header("4. Joystick Status")
    try:
        res = subprocess.run(["ls", "-l", "/dev/input/js0"], capture_output=True, text=True)
        if res.returncode == 0:
            print(f"  Joystick (/dev/input/js0): {GREEN}CONNECTED{RESET}")
        else:
            print(f"  Joystick (/dev/input/js0): {RED}NOT CONNECTED{RESET}")
    except Exception as e:
        print(f"  [ERROR] {e}")


if __name__ == "__main__":
    check_can_interface_stats()
    analyze_can_bus_traffic()
    check_joystick()
    print()
