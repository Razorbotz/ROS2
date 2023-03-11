import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Int16MultiArray
import serial

## @file
# @brief Node controlling excavation motors
# 
# This node listens for several topics, and sends the information
# to the ODrive excavation motors.  The topics this node subscribes to
# are as follows:
#
# \li \b excavationDrum
# \li \b excavationArm
# \li \b STOP
# \li \b GO
# 
# To read more about the nodes that publish these topics
# \see logic_node.cpp
# \see communication_node.cpp

class ArduinoNode(Node):
        def __init__(self):
                """! Initializes the program, sets subscriptions"""
                super().__init__('arduino_node')
                self.potentPublisher = self.create_publisher(Int16MultiArray, 'potentiometer_data', 5)
                timer_period = 0.5
                self.timer = self.create_timer(timer_period, self.getData)
                self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        
        def getData(self):
                try:
                        inputData = self.arduino.readline()
                        byteString = inputData.decode('utf-8')
                        self.data = byteString.strip().split(', ')
                        print(self.data)
                except:
                        self.arduino.close()
                self.publishData()
        
        
        def publishData(self):
                msg = Int16MultiArray()
                
                msg.data = [int(self.data[0]), int(self.data[1])]
                self.potentPublisher.publish(msg)
        

def main(args=None):
        rclpy.init(args=args)
        arduino_node = ArduinoNode()
        rclpy.spin(arduino_node)

        arduino_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
