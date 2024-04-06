#! /usr/bin/python3
import time
import rclpy
from rclpy.node import Node

from shared_msgs.msg import FinalThrustMsg, SPITestMsg

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.command_publisher = self.create_publisher(SPITestMsg, 'SPI_command', 10)
        logger = self.get_logger().info("PUBLISHER INITIALIZED")

class CommandSubscribe(Node):
    def __init__(self):
        super().__init__('test_thrust_spi')
        self.spi_sub = self.create_subscription(SPITestMsg, 'SPI_command', self.callback, 10)
        self.thrust_pub = self.create_publisher(FinalThrustMsg, 'final_thrust', 10)
        logger = self.get_logger().info("SUBSCRIBER INITIALIZED")

    def callback(self, msg):
        cmd_type = msg.type
        thrust = msg.thrust
        interval = msg.interval
        if cmd_type == "FULL":
            self.run_all(thrust)
        if cmd_type == "SEQ":
            self.run_seq(thrust, interval)

    def run_all(self, thrust):
        tcm = FinalThrustMsg()
        tcm.thrusters = bytearray([thrust] * 8)
        self.thrust_pub.publish(tcm)

    def run_seq(self, thrust, interval):
        tcm = FinalThrustMsg()
        for idx in range(8):
            thrusters = [127] * 8
            thrusters[idx] = thrust
            tcm.thrusters = bytearray(thrusters)
            time.sleep(interval)
        self.thrust_pub.publish(tcm)

def main(args=None):
    rclpy.init(args=args)
    pub = CommandPublisher()
    sub = CommandSubscribe()

    try:
        rclpy.spin(pub)
        rclpy.spin(sub)
    except KeyboardInterrupt:
        pass

    pub.destroy_node()
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# ros2 topic pub /SPI_command shared_msgs/SPITestMsg '{type: "SEQ", thrust: 140, interval: 3}'