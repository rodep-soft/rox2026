#!/usr/bin/env python3
"""
Bluetooth (RFCOMM) Controller Sender for PS5 DualSense.
Sends controller inputs via Bluetooth RFCOMM socket with auto-reconnect support.
"""

import time
import json
import argparse
from typing import Dict, Any


def get_controller_input(mock: bool = False) -> Dict[str, Any]:
    """
    コントローラーの入力を取得する関数。
    mock=True の場合はテスト用データを返します。
    """
    if mock:
        return {
            "axes": [0.0] * 8,
            "buttons": [0] * 14,
            "timestamp": time.time()
        }

    # TODO: Pygame や inputs 等を使って実際の PS5 コントローラー入力を取得する処理を実装
    return {
        "axes": [0.0] * 8,
        "buttons": [0] * 14,
        "timestamp": time.time()
    }


def main():
    parser = argparse.ArgumentParser(description="PS5 Controller Bluetooth Sender")
    parser.add_argument("--addr", type=str, default="00:11:22:33:44:55", help="Receiver Bluetooth MAC Address")
    parser.add_argument("--port", type=int, default=1, help="Receiver RFCOMM channel (default: 1)")
    parser.add_argument("--rate", type=float, default=50.0, help="Send rate in Hz (default: 50)")
    parser.add_argument("--mock", action="store_true", help="Send mock data for testing")
    # 追加: 再接続インターバルの引数定義
    parser.add_argument("--reconnect-interval", type=float, default=3.0, help="Wait time before reconnecting (seconds)")
    args = parser.parse_args()

    try:
        import bluetooth
    except ImportError:
        print("[BT Sender] Error: pybluez is required for Bluetooth transmission. Install via `pip install pybluez`.")
        return

    interval = 1.0 / args.rate

    # 外側の自動再接続ループ
    while True:
        sock = None
        try:
            print(f"[BT Sender] Connecting to {args.addr} on channel {args.port}...")
            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            sock.connect((args.addr, args.port))
            print("[BT Sender] Connected.")

            # 送信ループ
            while True:
                sample = get_controller_input(mock=args.mock)
                payload = (json.dumps(sample) + "\n").encode('utf-8')

                sock.send(payload)
                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n[BT Sender] Stopped by user.")
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            break  # Ctrl+C で安全に脱出

        except Exception as e:
            print(f"[BT Sender] Connection error or lost: {e}")
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

            print(f"[BT Sender] Reconnecting in {args.reconnect_interval} seconds...")
            time.sleep(args.reconnect_interval)


if __name__ == "__main__":
    main()
