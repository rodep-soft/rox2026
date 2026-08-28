#!/usr/bin/env python3
"""
Wi-Fi (UDP) Controller Sender for PS5 DualSense.
Reads controller inputs via evdev / Linux joystick and sends raw axes/buttons payload via UDP.
Features automatic socket recovery.
"""

import socket
import json
import time
import argparse
from typing import Dict, Any


def send_data_udp(sock: socket.socket, host: str, port: int, data: Dict[str, Any]) -> None:
    """Send JSON-encoded data to host:port over UDP with newline delimiter."""
    # パケット区切りのため改行コード (\n) を追加
    payload = (json.dumps(data) + "\n").encode('utf-8')
    sock.sendto(payload, (host, port))


def get_controller_payload(js, mock: bool = False) -> Dict[str, Any]:
    """コントローラー入力データまたはモックデータを取得"""
    if mock or js is None:
        return {
            "axes": [0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0],
            "buttons": [0] * 14,
            "timestamp": time.time()
        }

    import pygame
    pygame.event.pump()
    num_axes = js.get_numaxes()
    num_buttons = js.get_numbuttons()
    axes = [float(js.get_axis(i)) for i in range(num_axes)]
    buttons = [int(js.get_button(i)) for i in range(num_buttons)]

    return {
        "axes": axes,
        "buttons": buttons,
        "timestamp": time.time()
    }


def main():
    parser = argparse.ArgumentParser(description="PS5 Controller UDP Sender")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Receiver IP address (Wi-Fi)")
    parser.add_argument("--port", type=int, default=9999, help="Receiver UDP port (default: 9999)")
    parser.add_argument("--rate", type=float, default=50.0, help="Send rate in Hz (default: 50)")
    parser.add_argument("--mock", action="store_true", help="Send mock data for testing without controller")
    # 追加: リトライインターバルの引数定義
    parser.add_argument("--retry-interval", type=float, default=3.0, help="Wait time on error before retrying (seconds)")
    args = parser.parse_args()

    interval = 1.0 / args.rate
    print(f"[UDP Sender] Target: {args.host}:{args.port} | Rate: {args.rate} Hz")

    # pygameの初期化（--mock 指定時はスキップ）
    js = None
    if not args.mock:
        try:
            import pygame
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                js = pygame.joystick.Joystick(0)
                js.init()
                print(f"[UDP Sender] Connected to: {js.get_name()}")
            else:
                print("[UDP Sender] No joystick detected. Falling back to mock mode.")
        except ImportError:
            print("[UDP Sender] pygame not found. Falling back to mock mode.")

    # 外側の再接続・ソケット復旧ループ
    while True:
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print("[UDP Sender] UDP Socket ready. Starting transmission...")

            # 送信ループ
            while True:
                payload = get_controller_payload(js, mock=(args.mock or js is None))
                send_data_udp(sock, args.host, args.port, payload)
                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n[UDP Sender] Stopped by user.")
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            break  # Ctrl+C で安全に終了

        except Exception as e:
            print(f"[UDP Sender] Transmission error occurred: {e}")
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

            # 修正箇所: args.retry-interval -> args.retry_interval
            print(f"[UDP Sender] Retrying in {args.retry_interval} seconds...")
            time.sleep(args.retry_interval)


if __name__ == "__main__":
    main()
