import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time

class SerialResetNode(Node):
    def __init__(self):
        super().__init__('reset')

        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=9600,
                timeout=1
            )
            if self.serial_port.is_open:
                self.get_logger().info('Serial port opened successfully.')
            else:
                self.get_logger().error('Failed to open serial port.')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Serial exception: {e}')
            raise

        self.subscription = self.create_subscription(
            String,
            'reset_topic',
            self.reset_callback,
            10
        )

    def reset_callback(self, msg):
        self.get_logger().info(f"Received on 'reset_topic': '{msg.data}'")

        if self.serial_port.is_open:
            to_send = msg.data + "\n"
            self.serial_port.write(to_send.encode())
            self.get_logger().info(f"Sent over serial: '{to_send.strip()}'")
        else:
            self.get_logger().warn("Serial port is not open. Cannot send message.")

def main(args=None):
    rclpy.init(args=args)
    node = SerialResetNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
