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
        #GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(24, GPIO.OUT, initial=GPIO.LOW)
        #GPIO.setup(16, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(23, GPIO.OUT, initial=GPIO.LOW)

    
    def stepperCallback(self, msg):
        # Extend Stepper, 18 HIGH, 16 LOW
        if msg.data == 1:
            #GPIO.output(18, GPIO.HIGH)
            GPIO.output(24, GPIO.HIGH)
            #GPIO.output(16, GPIO.LOW)
            GPIO.output(23, GPIO.LOW)
            self.get_logger().info('Stepper: HIGH')
        # Retract Stepper, 18 HIGH, 16 HIGH
        elif msg.data == -1:
            #GPIO.output(18, GPIO.HIGH)
            GPIO.output(24, GPIO.HIGH)
            #GPIO.output(16, GPIO.HIGH)
            GPIO.output(23, GPIO.HIGH)
            self.get_logger().info('Stepper: LOW')
        # Do nothing, 18 LOW, 16 LOW
        else:
            #GPIO.output(18, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
            #GPIO.output(16, GPIO.LOW)
            GPIO.output(23, GPIO.LOW)

    
def main(args=None):
    rclpy.init(args=args)
    stepper_node = StepperNode()
    rclpy.spin(stepper_node)

    stepper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()