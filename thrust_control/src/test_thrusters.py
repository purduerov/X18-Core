#! /usr/bin/python3
import time
import rclpy
from rclpy.node import Node

from shared_msgs.msg import FinalThrustMsg, ThrusterTestMsg


class SPITest(Node):
    def __init__(self):
        super().__init__("test_thrust_spi")

        # initialize publishers and subscriber
        self.command_publisher = self.create_publisher(
            ThrusterTestMsg, "test_thrust", 10
        )
        self.spi_sub = self.create_subscription(
            ThrusterTestMsg, "test_thrust", self.callback, 10
        )
        self.thrust_pub = self.create_publisher(FinalThrustMsg, "final_thrust", 10)

        logger = self.get_logger().info("TEST INITIALIZATION")

    def callback(self, msg):
        # print("CALLBACK")
        self.get_logger().info("Test running")
        # initialize message variables
        cmd_type = msg.type
        thrust = msg.thrust
        interval = msg.interval

        # parse through command types
        if cmd_type == "FULL":
            self.run_all(thrust, interval)
        elif cmd_type == "SEQ":
            self.run_seq(thrust, interval)
        elif cmd_type == "RAMP":
            self.run_ramp(interval)
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

    def run_ramp(self, interval):
        print("RUNNING RAMP")
        tcm = FinalThrustMsg()

        thrusters = [127] * 8
        print(thrusters)
        tcm.thrusters = bytearray(thrusters)
        self.thrust_pub.publish(tcm)
        time.sleep(interval)

        thrust_val = 0
        while thrust_val < 256:
            thrusters = [thrust_val] * 8
            print(thrusters)
            tcm.thrusters = bytearray(thrusters)
            self.thrust_pub.publish(tcm)
            time.sleep(interval)
            thrust_val += 5

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

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# ros2 topic pub --once /test_thrust shared_msgs/ThrusterTestMsg '{type: "SEQ", thrust: 150, interval: 3}'
# ros2 topic pub --once /test_thrust shared_msgs/ThrusterTestMsg '{type: "FULL", thrust: 150, interval: 3}'
# ros2 topic pub --once /test_thrust shared_msgs/ThrusterTestMsg '{type: "RAMP", thrust: 0, interval: 3}'

# SEQ runs each thruster individually at specific thrust for the interval
# FULL runs all thrusters at specific thrust for the interval
# RAMP runs all thrusters starting from full back to full forward increasing value at interval
