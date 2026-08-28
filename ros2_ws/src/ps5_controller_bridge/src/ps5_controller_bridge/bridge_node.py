#!/usr/bin/env python3
"""
PS5 Controller Bridge Node for ROS 2.
Receives controller data via Wi-Fi (UDP) or Bluetooth (RFCOMM)
and publishes ROS 2 sensor_msgs/Joy messages with automatic reconnection support.
"""

import threading
import queue
import time
import socket
import json
import logging
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


# UDP receiver for Wi‑Fi (UDP)
class WifiReceiver(threading.Thread):
    def __init__(self, host: str, port: int, out_queue: queue.Queue):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.out_queue = out_queue
        self._stop_event = threading.Event()
        self.logger = logging.getLogger('WifiReceiver')
        self.sock = None

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                # ソケットの生成とバインド（切断・エラー時の再生成対応）
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.settimeout(2.0)
                self.sock.bind((self.host, self.port))
                
                while not self._stop_event.is_set():
                    try:
                        data, _addr = self.sock.recvfrom(65535)
                        if not data:
                            continue
                        message = json.loads(data.decode('utf-8').strip())
                        self.out_queue.put(('wifi', message))
                    except socket.timeout:
                        continue
                    except json.JSONDecodeError:
                        continue
            except Exception as e:
                if not self._stop_event.is_set():
                    self.logger.debug(f'Wi‑Fi UDP receive error: {e}')
                    time.sleep(1.0)
            finally:
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None

        self.logger.info('Wi‑Fi UDP receiver stopped')

    def stop(self):
        self._stop_event.set()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


# Receiver for Bluetooth (RFCOMM) with Auto-Reconnect & Line Buffer
class BtReceiver(threading.Thread):
    def __init__(self, bt_addr: str, port: int, out_queue: queue.Queue):
        super().__init__(daemon=True)
        self.bt_addr = bt_addr
        self.port = port
        self.out_queue = out_queue
        self._stop_event = threading.Event()
        self.logger = logging.getLogger('BtReceiver')
        self.sock = None

    def run(self) -> None:
        try:
            import bluetooth  # pybluez
        except ImportError:
            self.logger.error('pybluez not installed')
            return

        # 自動再接続ループ
        while not self._stop_event.is_set():
            try:
                self.logger.info(f'Connecting Bluetooth to {self.bt_addr}:{self.port}...')
                sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                sock.settimeout(5.0)
                self.sock = sock  # stop() 側から安全に参照できるよう保持
                
                sock.connect((self.bt_addr, self.port))
                self.logger.info('Bluetooth connected.')

                buffer = ""
                while not self._stop_event.is_set():
                    try:
                        data = sock.recv(1024)
                        if not data:
                            break  # 相手側から切断された場合
                        
                        buffer += data.decode('utf-8')
                        
                        # 改行区切りで受信メッセージを取り出してパース
                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            line = line.strip()
                            if line:
                                try:
                                    msg = json.loads(line)
                                    self.out_queue.put(('bt', msg))
                                except json.JSONDecodeError:
                                    continue
                    except socket.timeout:
                        continue

            except Exception as e:
                if not self._stop_event.is_set():
                    self.logger.debug(f'Bluetooth connection error: {e}. Reconnecting in 3s...')
                    time.sleep(3.0)
            finally:
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None

        self.logger.info('Bluetooth receiver stopped')

    def stop(self):
        self._stop_event.set()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


class BridgeNode(Node):
    def __init__(self):
        super().__init__('ps5_controller_bridge')
        self.declare_parameter('wifi_host', '0.0.0.0')
        self.declare_parameter('wifi_port', 9999)
        self.declare_parameter('bt_addr', '00:11:22:33:44:55')
        self.declare_parameter('bt_port', 1)
        self.declare_parameter('publish_rate', 50.0)

        self.publisher_ = self.create_publisher(Joy, 'ps5/joy', 10)
        self.queue = queue.Queue()
        self.active_source = 'wifi'

        # 受信スレッドの起動
        self.wifi_receiver = WifiReceiver(
            host=self.get_parameter('wifi_host').get_parameter_value().string_value,
            port=self.get_parameter('wifi_port').get_parameter_value().integer_value,
            out_queue=self.queue,
        )
        self.bt_receiver = BtReceiver(
            bt_addr=self.get_parameter('bt_addr').get_parameter_value().string_value,
            port=self.get_parameter('bt_port').get_parameter_value().integer_value,
            out_queue=self.queue,
        )
        self.wifi_receiver.start()
        self.bt_receiver.start()

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        timer_period = 1.0 / rate if rate > 0 else 0.02
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(f'Bridge node started at {rate} Hz, using Wi‑Fi as primary channel')

    def process(self):
        # キューの最新メッセージを取得（Wi-Fi優先）
        latest: Dict[str, Any] = {}
        while not self.queue.empty():
            source, data = self.queue.get_nowait()
            latest[source] = data

        if 'wifi' in latest:
            chosen = 'wifi'
            payload = latest['wifi']
        elif 'bt' in latest:
            chosen = 'bt'
            payload = latest['bt']
        else:
            return

        self.active_source = chosen
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 配列の型変換（安全なキャスト）
        joy_msg.axes = [float(x) for x in payload.get('axes', [])]
        joy_msg.buttons = [int(x) for x in payload.get('buttons', [])]
        
        self.publisher_.publish(joy_msg)
        self.get_logger().debug(f'Published Joy from {chosen}')

    def destroy_node(self):
        self.wifi_receiver.stop()
        self.bt_receiver.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
