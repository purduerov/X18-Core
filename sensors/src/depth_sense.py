#! /usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
import ms5837


class DepthSense(Node):
    def __init__(self):
        super().__init__('depth')
        self.publisher_ = self.create_publisher(Float64, "depth", 10)
        timer_period = (1.0 / 20.0)  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sensor = ms5837.MS5837(1)  # Initialize sensor on i2c bus 1
        init_success = self.sensor.init()  # Initializes with density of freshwater

        if init_success == 0:
            print("Depth could not be initialized... Exiting")
            exit(1)

    def timer_callback(self):
        msg = Float64()
        self.sensor.read()  # allows the sensor to read new data     -> maybe need to add this at the begining of each call to pull new data
        msg.data = self.sensor.depth()

        # publishes depth in meters at a rate of 20Hz
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pi_depth = DepthSense()

    rclpy.spin(pi_depth)

    pi_depth.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
