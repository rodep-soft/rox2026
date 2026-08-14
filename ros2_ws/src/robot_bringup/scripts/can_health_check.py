#!/usr/usr/bin/env python3
"""
CAN & System Health Checker for ROX2026 Robot (Professional Diagnostics Edition)
詳細プロレベルCAN診断：バスエラーカウンター、データレート、ノード別毎秒フレーム数(fps)、バス物理層トポロジー推測
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
    print(f"\n========================================================")
    print(f"  {title}")
    print(f"========================================================")


def check_can_interface_stats():
    print_header("1. Linux SocketCAN (can0) プロレベル物理＆統計診断")
    try:
        # ip -details -statistics link show can0
        res = subprocess.run(
            ["ip", "-details", "-s", "link", "show", "can0"], capture_output=True, text=True
        )
        output = res.stdout
        
        # 状態解析
        state_match = re.search(r"state\s+([A-Z\-]+)", output)
        bus_state = state_match.group(1) if state_match else "UNKNOWN"
        
        if bus_state == "UP":
            print("  [BUS 状態]       : \033[92m● UP (正常稼働中)\033[0m")
        elif "BUS-OFF" in bus_state or "BUS-OFF" in output:
            print("  [BUS 状態]       : \033[91m✖ BUS-OFF (通信遮断中！ターミネータ・短絡・ノイズ等要確認)\033[0m")
        elif "ERROR-PASSIVE" in output or "ERROR-WARNING" in output:
            print(f"  [BUS 状態]       : \033[93m▲ {bus_state} (警告：エラーパケット多発中)\033[0m")
        else:
            print(f"  [BUS 状態]       : \033[93m▲ {bus_state}\033[0m")

        # Bitrate & Restart-ms
        bitrate_match = re.search(r"bitrate\s+(\d+)", output)
        bitrate = bitrate_match.group(1) if bitrate_match else "不明"
        restart_match = re.search(r"restart-ms\s+(\d+)", output)
        restart_ms = restart_match.group(1) if restart_match else "未設定"
        print(f"  [ビットレート]   : {int(bitrate)//1000 if bitrate.isdigit() else bitrate} kbps (restart-ms: {restart_ms}ms)")

        # エラーカウンター (berr-counter TEC / REC)
        tec_rec_match = re.search(r"berr-counter\s+tec\s+(\d+)\s+rec\s+(\d+)", output)
        if tec_rec_match:
            tec, rec = int(tec_rec_match.group(1)), int(tec_rec_match.group(2))
            tec_str = f"\033[91m{tec}\033[0m" if tec > 96 else f"\033[92m{tec}\033[0m"
            rec_str = f"\033[91m{rec}\033[0m" if rec > 96 else f"\033[92m{rec}\033[0m"
            print(f"  [エラーカウンタ] : TxErr(TEC)={tec_str}, RxErr(REC)={rec_str}")

        # エラー統計 (bus-error, restarts, arbit-lost)
        restarts_match = re.search(r"restarts\s+(\d+)", output)
        bus_errors_match = re.search(r"bus-errors\s+(\d+)", output)
        if restarts_match or bus_errors_match:
            restarts = restarts_match.group(1) if restarts_match else "0"
            bus_errs = bus_errors_match.group(1) if bus_errors_match else "0"
            print(f"  [再起動/バスエラー] : 自動再起動回数={restarts}回, バスエラー検出={bus_errs}回")

        # TX / RX パケット数
        lines = output.splitlines()
        for i, line in enumerate(lines):
            if "RX: bytes" in line or "RX: bytes packets" in line:
                if i + 1 < len(lines):
                    print(f"  [累積受信 (RX)]  : {lines[i+1].strip()}")
            elif "TX: bytes" in line or "TX: bytes packets" in line:
                if i + 1 < len(lines):
                    print(f"  [累積送信 (TX)]  : {lines[i+1].strip()}")

    except Exception as e:
        print(f"  \033[91m[エラー] CANステータス取得失敗: {e}\033[0m")


def send_pings():
    """全登録ノードに対して探査フレーム（Ping）を送る"""
    time.sleep(0.3)
    for can_id in NODE_MAP.keys():
        try:
            # モータ/STM32にリード要求Pingを送信
            cmd = f"cansend can0 {can_id:03X}#0000"
            subprocess.run(cmd, shell=True, capture_output=True)
            time.sleep(0.02)
        except Exception:
            pass


def analyze_can_bus_traffic():
    print_header("2. リアルタイムCANトラフィック＆全ノードアクティブ診断 (2.0秒解析)")
    node_counts = defaultdict(int)
    unknown_nodes = defaultdict(int)
    total_frames = 0

    ping_thread = threading.Thread(target=send_pings)
    ping_thread.daemon = True

    try:
        # -L: log format (timestamp interface id#data)
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

                    # 自分が送ったPing（"0000"）はカウントから除外
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
        print("  \033[91m[エラー] candump が見つかりません。can-utils をインストールしてください。\033[0m")
        return
    except Exception as e:
        print(f"  [エラー] キャプチャ中に例外発生: {e}")
        return

    # バス負荷・フレームレート
    fps = total_frames / sample_duration
    # 通常の1Mbps CANフレーム (約111bit/frame)
    bus_load_pct = (fps * 111.0 / 1000000.0) * 100.0

    print(f"  [総受信フレーム]  : {total_frames} frames ({fps:.1f} fps)")
    print(f"  [推測バス負荷率]  : {bus_load_pct:.2f} %")
    print("\n  --- 各ノードの応答状況 & パケット更新レート ---")

    for can_id, name in NODE_MAP.items():
        count = node_counts[can_id]
        node_fps = count / sample_duration
        if count > 0:
            status_str = f"\033[92m● ONLINE ({node_fps:5.1f} Hz / {count:3d} pkts)\033[0m"
        else:
            status_str = f"\033[91m✕ NO RESPONSE (応答なし)\033[0m"
        
        print(f"  [0x{can_id:03X}] {name:<32} : {status_str}")

    if unknown_nodes:
        print("\n  --- 未定義IDの検知 ---")
        for u_id, count in unknown_nodes.items():
            u_fps = count / sample_duration
            print(f"  [0x{u_id:03X}] 未登録CANノード                     : {u_fps:.1f} Hz ({count} pkts)")

    # 物理トポロジー診断インサイト
    print_header("3. ロボットハードウェア・物理トポロジー診断インサイト")
    edulite_online = sum(1 for cid in [0x020, 0x021, 0x022, 0x023, 0x028, 0x038] if node_counts[cid] > 0)
    vesc_online = sum(1 for cid in [0x320, 0x321, 0x322] if node_counts[cid] > 0 or node_counts[0x100] > 0)
    stm32_online = node_counts[0x200] > 0 or node_counts[0x201] > 0

    if edulite_online == 0 and stm32_online and vesc_online:
        print("  \033[91m[診断結論] EduLite基板群 (0x20~0x38) のみが応答していません。\033[0m")
        print("             ➔ EduLite電源ライン(24V/12V)の断線、またはCANコネクタの抜け・接触不良を疑ってください。")
    elif edulite_online == 0 and not stm32_online and vesc_online:
        print("  \033[91m[診断結論] VESCのみONLINEで、EduLiteおよびSTM32が全滅しています。\033[0m")
        print("             ➔ VESC基板とSTM32/EduLite基板を結ぶ「中間CANケーブルの抜け」または「メイン制御電源OFF」です！\033[0m")
    elif edulite_online > 0 and edulite_online < 6:
        offline_nodes = [name for cid, name in NODE_MAP.items() if cid < 0x100 and node_counts[cid] == 0]
        print(f"  \033[93m[診断結論] EduLiteの一部ノード ({', '.join(offline_nodes)}) のみ応答がありません。\033[0m")
        print("             ➔ 該当モータのCANデイジーチェーンコネクタ、またはモータ基板個別のヒューズを確認してください。")
    elif edulite_online == 6 and vesc_online and stm32_online:
        print("  \033[92m[診断結論] 全ハードウェアノードが非常に良好に応答しています！物理層・電源共に問題ありません。\033[0m")
    else:
        print("  [診断結論] 一部のバス通信のみ生存しています。CANバスの終端抵抗(120Ω)やノイズの影響を確認してください。")


def check_joystick():
    print_header("4. コントローラー (Joystick) 認識診断")
    try:
        res = subprocess.run(["ls", "-l", "/dev/input/js0"], capture_output=True, text=True)
        if res.returncode == 0:
            print("  [ジョイパッド] : \033[92m● /dev/input/js0 検出 (認識OK)\033[0m")
        else:
            print("  [ジョイパッド] : \033[91m✕ /dev/input/js0 が見つかりません (コントローラー未接続)\033[0m")
    except Exception as e:
        print(f"  [エラー] : {e}")


if __name__ == "__main__":
    print("\n🤖 ROX2026 プロフェッショナルCAN＆ハードウェア診断ツール 🤖")
    check_can_interface_stats()
    analyze_can_bus_traffic()
    check_joystick()
    print("\n診断完了。\n")

