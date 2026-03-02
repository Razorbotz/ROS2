#!/usr/bin/env python3
import socket
import threading
import time
import json
from typing import Optional, Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def mac_to_int(mac: str) -> int:
    return int(mac.replace(":", ""), 16)


class BtRfcommServer(Node):
    def __init__(self):
        super().__init__("bt_rfcomm_server")

        self.declare_parameter("channel", 1)
        self.declare_parameter("rx_topic", "/bt/rx")
        self.declare_parameter("tx_topic", "/bt/tx")
        self.declare_parameter("my_mac", "")         # if empty, auto-detect from bluetoothctl show
        self.declare_parameter("peer_mac", "")       # required for tie-break coordination
        self.declare_parameter("allow_incoming", True)  # link-manager can toggle
        self.declare_parameter("enable_stdio", False)

        self.channel = int(self.get_parameter("channel").value)
        self.rx_topic = str(self.get_parameter("rx_topic").value)
        self.tx_topic = str(self.get_parameter("tx_topic").value)
        self.my_mac = str(self.get_parameter("my_mac").value).strip()
        self.peer_mac = str(self.get_parameter("peer_mac").value).strip()
        self.allow_incoming = bool(self.get_parameter("allow_incoming").value)
        self.enable_stdio = bool(self.get_parameter("enable_stdio").value)

        if not self.peer_mac:
            self.get_logger().warn("peer_mac is empty. Tie-break will be less deterministic.")
        if not self.my_mac:
            self.my_mac = self._detect_bt_mac()
            if self.my_mac:
                self.get_logger().info(f"Detected my_mac={self.my_mac}")
            else:
                self.get_logger().warn("Could not auto-detect my_mac; tie-break may not work.")

        self.rx_pub = self.create_publisher(String, self.rx_topic, 10)
        self.tx_sub = self.create_subscription(String, self.tx_topic, self._tx_cb, 10)

        self._server_sock: Optional[socket.socket] = None
        self._bt_clients: Set[socket.socket] = set()
        self._bt_clients_lock = threading.Lock()
        self._stop = threading.Event()

        # Start server + accept thread
        self._start_server()

        # Optional stdin broadcast for quick manual testing from server side
        if self.enable_stdio:
            threading.Thread(target=self._stdin_loop, daemon=True).start()

        # Timer to enforce allow_incoming toggle
        self.create_timer(1.0, self._enforce_allow_incoming)

        self.get_logger().info(
            f"RFCOMM server listening on channel={self.channel}, rx_topic={self.rx_topic}, tx_topic={self.tx_topic}"
        )

    def _detect_bt_mac(self) -> str:
        # Try bluetoothctl show (works on Ubuntu/BlueZ)
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

    def _start_server(self):
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.bind(("00:00:00:00:00:00", self.channel))
        s.listen(7)
        self._server_sock = s
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                client, addr = self._server_sock.accept()
            except OSError:
                break

            # If incoming is currently not allowed, immediately drop
            if not self.allow_incoming:
                try:
                    client.close()
                except Exception:
                    pass
                continue

            self.get_logger().info(f"Incoming RFCOMM connection from {addr}")
            with self._bt_clients_lock:
                self._bt_clients.add(client)

            try:
                client.send(b'{"type":"server_hello"}\n')
            except Exception:
                pass
            threading.Thread(target=self._client_rx_loop, args=(client, addr), daemon=True).start()

    def _client_rx_loop(self, sock: socket.socket, addr):
        buf = ""
        try:
            while not self._stop.is_set():
                data = sock.recv(1024)
                if not data:
                    self.get_logger().warn(f"Client {addr} disconnected")
                    return
                buf += data.decode(errors="replace")

                # Newline-delimited JSON (or plain text) framing
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    # Publish raw line to ROS
                    msg = String()
                    msg.data = line
                    self.rx_pub.publish(msg)

                    # Also log
                    self.get_logger().info(f"<< [{addr}] {line}")

        except OSError as e:
            self.get_logger().warn(f"RX error from {addr}: {e}")
        finally:
            with self._bt_clients_lock:
                self._bt_clients.discard(sock)
            try:
                sock.close()
            except Exception:
                pass

    def _tx_cb(self, msg: String):
        # Broadcast to all connected clients (normally you’ll have at most one after tie-break)
        payload = (msg.data.strip() + "\n").encode()
        dead = []
        with self._bt_clients_lock:
            for c in list(self._bt_clients):
                try:
                    c.send(payload)
                except OSError:
                    dead.append(c)
            for c in dead:
                self._bt_clients.discard(c)
                try:
                    c.close()
                except Exception:
                    pass

    def _stdin_loop(self):
        import sys
        while not self._stop.is_set():
            sys.stdout.write("server>> ")
            sys.stdout.flush()
            line = sys.stdin.readline()
            if not line:
                return
            line = line.strip()
            if not line:
                continue
            # publish to tx_topic so it goes out over the same path as ROS messages
            m = String()
            m.data = line
            self._tx_cb(m)

    def _enforce_allow_incoming(self):
        # refresh parameter in case changed at runtime
        self.allow_incoming = bool(self.get_parameter("allow_incoming").value)
        if not self.allow_incoming:
            # Drop any existing clients if incoming is disallowed
            with self._bt_clients_lock:
                for c in list(self._bt_clients):
                    try:
                        c.close()
                    except Exception:
                        pass
                self._bt_clients.clear()

    def destroy_node(self):
        self._stop.set()
        try:
            if self._server_sock:
                self._server_sock.close()
        except Exception:
            pass
        with self._bt_clients_lock:
            for c in list(self._bt_clients):
                try:
                    c.close()
                except Exception:
                    pass
            self._bt_clients.clear()
        super().destroy_node()


def main():
    rclpy.init()
    node = BtRfcommServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()