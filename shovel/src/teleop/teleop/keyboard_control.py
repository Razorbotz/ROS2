import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import sys
import select
import termios
import tty
import time
import math

# --- TUNING ---
SPEED_LINEAR = 5.0
SPEED_TURN = 5.0
MSG_TYPE = Float64MultiArray

SIDE_DURATION = 5.0
TURN_TOLERANCE = 0.1
KP = 0.5

# Topics
IMU_TOPIC = '/my_robot/zed2i/imu/data'
CONTROLLER_TOPIC_FMT = '/falcon_{}_controller/commands'

settings = termios.tcgetattr(sys.stdin)

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key = sys.stdin.read(2)
            if key == '[A': return 'up'
            elif key == '[B': return 'down'
            elif key == '[C': return 'right'
            elif key == '[D': return 'left'
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def quaternion_to_yaw(q):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Publishers
        self.pub_10 = self.create_publisher(MSG_TYPE, CONTROLLER_TOPIC_FMT.format(10), 10)
        self.pub_11 = self.create_publisher(MSG_TYPE, CONTROLLER_TOPIC_FMT.format(11), 10)
        self.pub_12 = self.create_publisher(MSG_TYPE, CONTROLLER_TOPIC_FMT.format(12), 10)
        self.pub_13 = self.create_publisher(MSG_TYPE, CONTROLLER_TOPIC_FMT.format(13), 10)

        # IMU
        self.current_yaw = 0.0
        self.create_subscription(Imu, IMU_TOPIC, self.imu_callback, 10)

        # State for smoothing
        self.target_left = 0.0
        self.target_right = 0.0
        self.last_key_time = 0.0

        self.get_logger().info("Smoothed Teleop Ready. Press 's' for Auto.")

    def imu_callback(self, msg):
        self.current_yaw = quaternion_to_yaw(msg.orientation)

    def publish_cmd(self, left, right):
        msg_l = Float64MultiArray()
        msg_l.data = [float(left)]
        msg_r = Float64MultiArray()
        msg_r.data = [float(right)]

        self.pub_10.publish(msg_l)
        self.pub_12.publish(msg_l)
        self.pub_11.publish(msg_r)
        self.pub_13.publish(msg_r)

    def turn_90_degrees(self):
        rclpy.spin_once(self, timeout_sec=0)
        start_yaw = self.current_yaw
        target_yaw = normalize_angle(start_yaw + (math.pi / 2.0))
        self.get_logger().info(f"Target Yaw: {target_yaw:.2f}")

        timeout_start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            
            if time.time() - timeout_start > 5.0:
                self.publish_cmd(0.0, 0.0)
                break

            error = normalize_angle(target_yaw - self.current_yaw)
            
            if abs(error) < TURN_TOLERANCE:
                self.publish_cmd(0.0, 0.0)
                break
            
            speed = error * KP
            speed = max(min(speed, SPEED_TURN), -SPEED_TURN)
            
            if speed > 0 and speed < 1.0: speed = 1.0
            if speed < 0 and speed > -1.0: speed = -1.0

            self.publish_cmd(-speed, speed)
            time.sleep(0.01)

    def run_square_routine(self):
        self.get_logger().info("--- Auto Start ---")
        for i in range(4):
            start_time = time.time()
            while time.time() - start_time < SIDE_DURATION:
                self.publish_cmd(SPEED_LINEAR, SPEED_LINEAR)
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(0.01)
            self.get_logger().info("Turning")
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.5)
            self.turn_90_degrees()
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key()
            
            current_time = time.time()

            if key == 's':
                node.run_square_routine()
                node.target_left = 0.0
                node.target_right = 0.0
                continue
            elif key == 'q':
                break
            elif key == '\x03':
                break
            
            if key in ['up', 'down', 'left', 'right']:
                node.last_key_time = current_time
                
                if key == 'up':
                    node.target_left = SPEED_LINEAR
                    node.target_right = SPEED_LINEAR
                elif key == 'down':
                    node.target_left = -SPEED_LINEAR
                    node.target_right = -SPEED_LINEAR
                elif key == 'left':
                    node.target_left = -SPEED_TURN
                    node.target_right = SPEED_TURN
                elif key == 'right':
                    node.target_left = SPEED_TURN
                    node.target_right = -SPEED_TURN
            
            if current_time - node.last_key_time > 0.2:
                node.target_left = 0.0
                node.target_right = 0.0

            node.publish_cmd(node.target_left, node.target_right)

    except Exception as e:
        print(e)
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()