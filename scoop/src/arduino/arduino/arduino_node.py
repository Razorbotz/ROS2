import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Int16MultiArray
import serial

## @file
# @brief Node for publishing the potentiometer data
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
                try:
                        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
                        self.arduinoInitialized = True
                except:
                        try:
                                self.arduino = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
                                self.arduinoInitialized = True
                        except:
                                self.arduino = None
                                self.arduinoInitialized = False
        
        def getData(self):
                try:
                        inputData = self.arduino.readline()
                        byteString = inputData.decode('utf-8')
                        self.data = byteString.strip().split(', ')
                        print(self.data)
                except:
                        if(self.arduinoInitialized):
                                self.arduino.close()
                self.publishData()
        
        
        def publishData(self):
                msg = Int16MultiArray()
                try:
                        msg.data = [int(self.data[0]), int(self.data[1]), int(self.data[2])]
                except:
                        msg.data = [-1, -1, -1]
                self.potentPublisher.publish(msg)
        

def main(args=None):
        rclpy.init(args=args)
        arduino_node = ArduinoNode()
        rclpy.spin(arduino_node)

        arduino_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
