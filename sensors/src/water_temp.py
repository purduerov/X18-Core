#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
# Library for the temperature sensor
# More info: https://github.com/bluerobotics/tsys01-python
import tsys01

class WaterTemp(Node):
    def __init__(self):
        super().__init__('water_temp')
        self.publisher_ = self.create_publisher(Float32, "water_temp", 10)
        timer_period = 1 #1
        self.timer = self.create_timer(timer_period, self.timer_callback)  
        # Set up i2c for the temperature sensor
        try:
            self.sensor = tsys01.TSYS01()
            self.sensor.init()
        except:
            self.get_logger().info("Temperature sensor not found")
            self.sensor = None
            return

    
    def timer_callback(self):
        msg = Float32()
        # Default units are Celsius
        temp = self.sensor.read()
        # Uncomment for Fahrenheit
        # temp = self.sensor.temperature(tsys01.UNITS_Farenheit)
        msg.data = temp
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args = args)
    water_temp = WaterTemp()
    rclpy.spin(water_temp)
    water_temp.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()