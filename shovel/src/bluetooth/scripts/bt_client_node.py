#!/usr/bin/env python3
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def mac_to_int(mac: str) -> int:
    return int(mac.replace(":", ""), 16)


class BtRfcommClient(Node):
    def __init__(self):
        super().__init__("bt_rfcomm_client")

        self.declare_parameter("peer_mac", "")      # remote server MAC (peer robot)
        self.declare_parameter("channel", 1)
        self.declare_parameter("rx_topic", "/bt/rx")
        self.declare_parameter("tx_topic", "/bt/tx")
        self.declare_parameter("my_mac", "")        # if empty, auto-detect
        self.declare_parameter("enable_stdio", False)

        self.peer_mac = str(self.get_parameter("peer_mac").value).strip()
        self.channel = int(self.get_parameter("channel").value)
        self.rx_topic = str(self.get_parameter("rx_topic").value)
        self.tx_topic = str(self.get_parameter("tx_topic").value)
        self.my_mac = str(self.get_parameter("my_mac").value).strip()
        self.enable_stdio = bool(self.get_parameter("enable_stdio").value)

        if not self.my_mac:
            self.my_mac = self._detect_bt_mac()
            if self.my_mac:
                self.get_logger().info(f"Detected my_mac={self.my_mac}")
            else:
                self.get_logger().warn("Could not auto-detect my_mac; tie-break may not work.")

        if not self.peer_mac:
            raise RuntimeError("peer_mac parameter is required for bt_rfcomm_client")

        # Tie-break: only one side actively dials
        try:
            self._i_dial = mac_to_int(self.my_mac) > mac_to_int(self.peer_mac)
        except Exception:
            # fallback if parsing fails
            self._i_dial = True

        role = "DIALER" if self._i_dial else "PASSIVE"
        self.get_logger().info(f"Tie-break role={role} (my_mac={self.my_mac}, peer_mac={self.peer_mac})")

        self.rx_pub = self.create_publisher(String, self.rx_topic, 10)
        self.tx_sub = self.create_subscription(String, self.tx_topic, self._tx_cb, 10)

        self._sock: Optional[socket.socket] = None
        self._sock_lock = threading.Lock()
        self._stop = threading.Event()

        # Heartbeat
        self._seq = 0
        self.create_timer(1.0, self._heartbeat_tick)

        # Start RX thread
        threading.Thread(target=self._rx_loop, daemon=True).start()

        # Optional stdin sender for manual testing from client side
        if self.enable_stdio:
            threading.Thread(target=self._stdin_loop, daemon=True).start()

        # Start connect loop if dialer
        if self._i_dial:
            threading.Thread(target=self._connect_loop, daemon=True).start()

    def _detect_bt_mac(self) -> str:
        import subprocess
        try:
            out = subprocess.check_output(["bluetoothctl", "show"], text=True, stderr=subprocess.STDOUT)
            for line in out.splitlines():
                line = line.strip()
                if line.lower().startswith("controller "):
                    return line.split()[1].strip()
        except Exception:
            pass
        return ""

    def _connect_loop(self):
        retry = 1.0
        while not self._stop.is_set():
            if self._is_connected():
                time.sleep(0.5)
                continue

            try:
                s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.get_logger().info(f"Connecting to peer {self.peer_mac} ch={self.channel} ...")
                s.connect((self.peer_mac, self.channel))
                with self._sock_lock:
                    # close old if any
                    if self._sock is not None:
                        try:
                            self._sock.close()
                        except Exception:
                            pass
                    self._sock = s
                self.get_logger().info("RFCOMM connected (outgoing).")
                retry = 1.0
            except OSError as e:
                self.get_logger().warn(f"Connect failed: {e} (retry in {retry:.1f}s)")
                try:
                    s.close()
                except Exception:
                    pass
                time.sleep(retry)
                retry = min(10.0, retry * 1.5)

    def _is_connected(self) -> bool:
        with self._sock_lock:
            return self._sock is not None

    def _send_line(self, line: str):
        payload = (line.strip() + "\n").encode()
        with self._sock_lock:
            s = self._sock
        if s is None:
            return
        try:
            s.send(payload)
        except OSError as e:
            self.get_logger().warn(f"Send failed (dropping connection): {e}")
            with self._sock_lock:
                try:
                    if self._sock:
                        self._sock.close()
                except Exception:
                    pass
                self._sock = None

    def _tx_cb(self, msg: String):
        self._send_line(msg.data)

    def _rx_loop(self):
        buf = ""
        while not self._stop.is_set():
            with self._sock_lock:
                s = self._sock
            if s is None:
                time.sleep(0.2)
                continue
            try:
                data = s.recv(1024)
                if not data:
                    self.get_logger().warn("Peer disconnected (outgoing).")
                    with self._sock_lock:
                        try:
                            if self._sock:
                                self._sock.close()
                        except Exception:
                            pass
                        self._sock = None
                    continue

                buf += data.decode(errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    out = String()
                    out.data = line
                    self.rx_pub.publish(out)
                    self.get_logger().info(f"<< {line}")

            except OSError as e:
                self.get_logger().warn(f"RX error (dropping connection): {e}")
                with self._sock_lock:
                    try:
                        if self._sock:
                            self._sock.close()
                    except Exception:
                        pass
                    self._sock = None

    def _heartbeat_tick(self):
        self._seq += 1
        hb = f'{{"type":"hb","seq":{self._seq},"t":{time.time():.3f}}}'
        self._send_line(hb)

    def _stdin_loop(self):
        import sys
        while not self._stop.is_set():
            sys.stdout.write("client>> ")
            sys.stdout.flush()
            line = sys.stdin.readline()
            if not line:
                return
            line = line.strip()
            if not line:
                continue
            # Publish to tx_topic so it uses same path as ROS messages
            msg = String()
            msg.data = line
            self._tx_cb(msg)

    def destroy_node(self):
        self._stop.set()
        with self._sock_lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
        super().destroy_node()


def main():
    rclpy.init()
    node = BtRfcommClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()