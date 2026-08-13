#!/usr/bin/env python3
"""
CAN & System Health Checker for ROX2026 Robot
一発で全CANノード（足回り・キッカー・ドリブル・STM32）の生存確認と通信エラーを自動診断するツール
"""

import subprocess
import time
import sys

# 監視対象の CAN ID マッピング
NODE_MAP = {
    0x20: "Front Left Wheel (EduLite)",
    0x21: "Front Right Wheel (EduLite)",
    0x22: "Rear Left Wheel (EduLite)",
    0x23: "Rear Right Wheel (EduLite)",
    0x28: "Spring / Kicker (EduLite)",
    0x38: "Dribble Motor (EduLite)",
    0x200: "STM32 Status / RX",
    0x201: "STM32 Control / TX",
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
        
        # txqueuelen 確認
        if "qlen 1000" in output or "qlen" in output:
            for line in output.splitlines():
                if "qlen" in line:
                    print(f"  [送信キュー] : {line.strip()}")
    except Exception as e:
        print(f"  \033[91m[エラー] ip コマンドの実行に失敗: {e}\033[0m")

def check_can_nodes():
    print_header("2. CAN バス上ノードのリアルタイム応答テスト (3秒間キャプチャ)")
    print("  CAN バスをリスニング中...")
    detected_ids = set()
    
    try:
        # timeout 3秒で candump を実行
        proc = subprocess.Popen(["candump", "can0"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        start_time = time.time()
        while time.time() - start_time < 3.0:
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
        print(f"  [キャプチャ完了]")

    print("\n  --- CAN ノード応答チェック結果 ---")
    for can_id, name in NODE_MAP.items():
        if can_id in detected_ids:
            print(f"  [0x{can_id:03X}] {name:<30} : \033[92m● ONLINE (応答あり)\033[0m")
        else:
            print(f"  [0x{can_id:03X}] {name:<30} : \033[91m✕ NO RESPONSE (応答なし/電源切れ/断線?)\033[0m")

    # 未定義のIDが検出された場合
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
    print("\n🤖 ROX2026 ロボット一括デバッグ・全診断ツール 🤖")
    check_can_interface()
    check_can_nodes()
    check_joystick()
    print("\n診断完了。\n")
