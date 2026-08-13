#!/usr/bin/env python3
"""
CAN & System Health Checker for ROX2026 Robot (Active Ping Edition)
アクティブ探査（Can Ping）付き：全CANノード（足回り・キッカー・ドリブル・VESC・STM32）の生存確認
"""

import subprocess
import time
import threading

NODE_MAP = {
    0x020: "Front Left Wheel (EduLite)",
    0x021: "Front Right Wheel (EduLite)",
    0x022: "Rear Left Wheel (EduLite)",
    0x023: "Rear Right Wheel (EduLite)",
    0x028: "Spring / Kicker (EduLite)",
    0x038: "Dribble Motor (EduLite)",
    0x200: "STM32 Status / RX",
    0x201: "STM32 Control / TX",
    0x320: "Upper Belt Motor (VESC)",
    0x321: "Under Belt Motor (VESC)",
}

def print_header(title):
    print(f"\n========================================================")
    print(f"  {title}")
    print(f"========================================================")

def check_can_interface():
    print_header("1. Linux SocketCAN (can0) ステータス診断")
    try:
        res = subprocess.run(["ip", "-details", "link", "show", "can0"], capture_output=True, text=True)
        output = res.stdout
        if "state UP" in output:
            print("  [can0 状態] : \033[92mUP (正常稼働中)\033[0m")
        elif "BUS-OFF" in output:
            print("  [can0 状態] : \033[91mBUS-OFF (通信遮断中！restartが必要です)\033[0m")
        else:
            print(f"  [can0 状態] : \033[93m{output.splitlines()[0] if output else 'DOWN'}\033[0m")
        
        for line in output.splitlines():
            if "qlen" in line:
                print(f"  [送信キュー] : {line.strip()}")
    except Exception as e:
        print(f"  \033[91m[エラー] ip コマンドの実行に失敗: {e}\033[0m")

def send_pings():
    """静観ノードに対して探査フレーム（Ping）を送る"""
    time.sleep(0.5)
    for can_id in NODE_MAP.keys():
        try:
            # モータ/STM32にリード要求Pingを送信
            cmd = f"cansend can0 {can_id:03X}#0000"
            subprocess.run(cmd, shell=True, capture_output=True)
            time.sleep(0.05)
        except Exception:
            pass

def check_can_nodes():
    print_header("2. CAN バス上ノードのアクティブ探査テスト (Ping探査中...)")
    detected_ids = set()
    
    # Ping送信スレッド開始
    ping_thread = threading.Thread(target=send_pings)
    ping_thread.daemon = True
    
    try:
        proc = subprocess.Popen(["candump", "can0"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        ping_thread.start()
        
        start_time = time.time()
        while time.time() - start_time < 2.5:
            line = proc.stdout.readline()
            if not line:
                break
            parts = line.split()
            if len(parts) >= 3:
                try:
                    can_id_str = parts[1]
                    can_id = int(can_id_str, 16)
                    detected_ids.add(can_id)
                except ValueError:
                    pass
        proc.terminate()
        proc.wait(timeout=1.0)
    except FileNotFoundError:
        print("  \033[91m[エラー] candump コマンドが見つかりません。sudo apt install can-utils を実行してください。\033[0m")
        return
    except Exception as e:
        pass

    print("  --- CAN ノードアクティブ探査結果 ---")
    for can_id, name in NODE_MAP.items():
        if can_id in detected_ids:
            print(f"  [0x{can_id:03X}] {name:<30} : \033[92m● ONLINE (応答あり)\033[0m")
        else:
            print(f"  [0x{can_id:03X}] {name:<30} : \033[91m✕ NO RESPONSE (応答なし/電源切れ/断線?)\033[0m")

    extra_ids = [hex(i) for i in detected_ids if i not in NODE_MAP]
    if extra_ids:
        print(f"\n  [その他の検出ID] : {', '.join(extra_ids)}")

def check_joystick():
    print_header("3. コントローラー (Joystick) 認識診断")
    try:
        res = subprocess.run(["ls", "-l", "/dev/input/js0"], capture_output=True, text=True)
        if res.returncode == 0:
            print("  [ジョイパッド] : \033[92m/dev/input/js0 検出 (認識OK)\033[0m")
        else:
            print("  [ジョイパッド] : \033[91m/dev/input/js0 が見つかりません (コントローラー未接続)\033[0m")
    except Exception as e:
        print(f"  [エラー] : {e}")

if __name__ == "__main__":
    print("\n🤖 ROX2026 ロボットアクティブ診断ツール 🤖")
    check_can_interface()
    check_can_nodes()
    check_joystick()
    print("\n診断完了。\n")
