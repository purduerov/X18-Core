#! /usr/bin/python3
import time
import rclpy
from rclpy.node import Node

from shared_msgs.msg import FinalThrustMsg, SPITestMsg

class SPITest(Node):
    def __init__(self):
        super().__init__('test_thrust_spi')

        # initialize publishers and subscriber
        self.command_publisher = self.create_publisher(SPITestMsg, 'SPI_command', 10)
        self.spi_sub = self.create_subscription(SPITestMsg, 'SPI_command', self.callback, 10)
        self.thrust_pub = self.create_publisher(FinalThrustMsg, 'final_thrust', 10)

        logger = self.get_logger().info("TEST INITIALIZATION")

    def callback(self, msg):
        # print("CALLBACK")

        # initialize message variables
        cmd_type = msg.type
        thrust = msg.thrust
        interval = msg.interval

        # parse through command types
        if cmd_type == "FULL":
            self.run_all(thrust, interval)
        elif cmd_type == "SEQ":
            self.run_seq(thrust, interval)
        else:
            print("COMMAND TYPE NOT FOUND: 'FULL' or 'SEQ'")

    def run_all(self, thrust, interval):
        print("RUNNING ALL")
        tcm = FinalThrustMsg()

        # set all thrusters to value of thrust for interval seconds
        thrusters = [thrust] * 8
        print(thrusters)
        tcm.thrusters = bytearray(thrusters)
        self.thrust_pub.publish(tcm)
        time.sleep(interval)

        # zero out thruster
        thrusters = [127] * 8
        print(thrusters)
        tcm.thrusters = bytearray(thrusters)
        self.thrust_pub.publish(tcm)

    def run_seq(self, thrust, interval):
        print("RUNNING SEQUENTIAL")
        tcm = FinalThrustMsg()

        # individually set each thruster to value of thrust for for interval seconds
        for idx in range(8):
            thrusters = [127] * 8
            thrusters[idx] = thrust
            print(thrusters)
            tcm.thrusters = bytearray(thrusters)
            self.thrust_pub.publish(tcm)
            time.sleep(interval)

        # zero out the thrusters
        thrusters = [127] * 8
        print(thrusters)
        tcm.thrusters = bytearray(thrusters)
        self.thrust_pub.publish(tcm)

def main(args=None):
    rclpy.init(args=args)
    node = SPITest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# ros2 topic pub --once /SPI_command shared_msgs/SPITestMsg '{type: "SEQ", thrust: 150, interval: 3}'
# ros2 topic pub --once /SPI_command shared_msgs/SPITestMsg '{type: "FULL", thrust: 150, interval: 3}'
