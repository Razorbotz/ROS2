import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

from std_msgs.msg import Float32

class StepperNode(Node):
    def __init__(self):
        super().__init__('stepper_node')
        self.get_logger().info('stepper_node')
        self.servoStateSubscriber = self.create_subscription(Float32, 'stepper_speed', self.stepperCallback, 10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)

    
    def stepperCallback(self, msg):
        if msg.data == 1:
            print("1")
        elif msg.data == -1:
            print("-1")

    
def main(args=None):
    rclpy.init(args=args)
    stepper_node = StepperNode()
    rclpy.spin(stepper_node)

    stepper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()