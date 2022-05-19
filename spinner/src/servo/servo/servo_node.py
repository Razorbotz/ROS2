import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Bool

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.get_logger().info('servo_node')
        self.servoStateSubscriber = self.create_subscription(Bool, 'servo_state', self.servoStateCallback, 10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)

        self.currentValue = False
        
    def servoStateCallback(self, msg):
        if msg.data == True:
            self.get_logger().info("Setting Pin 12 HIGH")
            GPIO.output(18, GPIO.HIGH)
            self.currentValue = True
        else:
            self.get_logger().info("Setting Pin 12 LOW")
            GPIO.output(18, GPIO.LOW)
            self.currentValue = False

def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoNode
    rclpy.spin(servo_node)

    servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()