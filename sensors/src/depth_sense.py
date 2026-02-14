#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from shared_msgs.msg import SensorCoordination
# Library for the depth sensor
# More info: https://github.com/bluerobotics/ms5837-python
import ms5837

IMU_COMPLETE = [True, False, False, True]
DEPTH_READING = [False, True, False, False]
DEPTH_COMPLETE = [False, True, False, True]

class DepthSense(Node):
    def __init__(self):
        super().__init__("depth")
        self.publisher_ = self.create_publisher(Float64, "depth", 10)
        self.coord_publisher_ = self.create_publisher(SensorCoordination, "sensor_coordination", 10)
        self.coord_subscriber = self.create_subscription(SensorCoordination, "sensor_coordination", self.coord_callback, 10)
        self.i2c_status = []
        
        timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        try:
            self.sensor = ms5837.MS5837(1)  # Initialize sensor on i2c bus 1
            self.sensor.init()  # Initializes with density of freshwater
            self.get_logger().info("Depth sensor connected")
        except:
            self.get_logger().info("Depth sensor not found")
            self.sensor = None
            exit(1)

    def timer_callback(self):
        msg = Float64()
        if self.i2c_status == IMU_COMPLETE:
            self.i2c_status = DEPTH_READING
            self.coord_publisher_.publish(self.i2c_status)
            self.sensor.read()  # allows the sensor to read new data     -> maybe need to add this at the begining of each call to pull new data
            msg.data = self.sensor.depth()
            self.publisher_.publish(msg)
            self.get_logger().info(f"Depth: {round(msg.data, 3)} km")
        elif self.i2c_status == DEPTH_READING:
            self.i2c_status = DEPTH_COMPLETE
            self.coord_publisher_.publish(self.i2c_status)



        # publishes depth in meters at a rate of 20Hz (no it doesnt)
        # self.publisher_.publish(msg)
        # self.get_logger().info(f"Depth: {round(msg.data, 3)} km")
    
    def coord_callback(self, msg):
        self.i2c_status = list(msg.i2c_status)

def main(args=None):
    rclpy.init(args=args)
    pi_depth = DepthSense()

    rclpy.spin(pi_depth)

    pi_depth.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
