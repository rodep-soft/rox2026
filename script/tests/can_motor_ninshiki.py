import can

bus = can.interface.Bus(
    channel='can0',
    bustype='socketcan',
    bitrate=500000
)

print("受信待機中...（Ctrl+Cで終了）")

try:
    while True:
        msg = bus.recv(timeout=1.0)  # 1秒待つ
        if msg is not None:
            print(f"ID: {hex(msg.arbitration_id)}  Data: {msg.data}")
        else:
            print("受信なし")
except KeyboardInterrupt:
            print("終了")
