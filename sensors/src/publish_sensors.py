#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32
# Library for the depth sensor
# More info: https://github.com/bluerobotics/ms5837-python
import ms5837
import ADT7410

class PublishSensors(Node):
    def __init__(self):
        super().__init__("sensors")
        self.depth_publisher_ = self.create_publisher(Float64, "depth", 10)
        self.temp_publisher_ = self.create_publisher(Float32, "hat_temp", 10)

        
        timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        try:
            self.sensor0 = ms5837.MS5837(1)  # Initialize sensor on i2c bus 1
            self.sensor0.init()  # Initializes with density of freshwater
            self.get_logger().info("Depth sensor connected")
        except:
            self.get_logger().info("Depth sensor not found")
            self.sensor0 = None
            exit(1)
        self.sensor1 = ADT7410() #hat temp
        self.sensor1.init()

    def timer_callback(self):
        depth_msg = Float64()
        temp_msg = Float32()
        
        self.sensor0.read()  # allows the sensor to read new data     -> maybe need to add this at the begining of each call to pull new data
        depth_msg.data = self.sensor0.depth()
        self.depth_publisher_.publish(depth_msg)
        self.get_logger().info(f"Depth: {round(depth_msg.data, 3)} km")
        temp_msg.data = self.sensor1.read_temperature()
        self.temp_publisher_.publish(temp_msg)
        self.get_logger().info(f"Hat Temp: {round(temp_msg.data, 3)}")

        # publishes depth in meters at a rate of 20Hz (no it doesnt)
        # self.publisher_.publish(msg)
        # self.get_logger().info(f"Depth: {round(msg.data, 3)} km")

def main(args=None):
    rclpy.init(args=args)
    pub_sensors = PublishSensors()

    rclpy.spin(pub_sensors)

    pub_sensors.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
