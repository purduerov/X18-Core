#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gpiozero import CPUTemperature
from std_msgs.msg import String
from std_msgs.msg import Float32


class PiTemp(Node):
    def __init__(self):
        super().__init__("pi_temp")
        self.publisher_ = self.create_publisher(Float32, "pi_temp", 10)

        timer_period = 1  # 1 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        cpu = CPUTemperature()
        msg.data = cpu.temperature
        self.get_logger().info(f"PI TEMP: {cpu}")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pi_temp = PiTemp()
    rclpy.spin(pi_temp)
    pi_temp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
