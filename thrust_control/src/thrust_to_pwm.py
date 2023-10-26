#! /usr/bin/python3
import signal
import time

import adafruit_pca9685
import board
import busio
import rclpy
from rclpy.node import Node

from shared_msgs.msg import FinalThrustMsg


zero_thrust = [127, 127, 127, 127, 127, 127, 127, 127]  # power of thrusters --> 127 is neutral


class ThrustToPWMNode(Node):
    prev_thrust = [127, 127, 127, 127, 127, 127, 127, 127]  # power of thrusters --> 127 is neutral

    def __init__(self):
        super().__init__('thrust_to_pwm')

        self.sub = self.create_subscription(
            FinalThrustMsg,
            'final_thrust',
            self.message_received,
            10
        )

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(i2c)
        self.pca.frequency = 500

        # TODO: why run thrusters on startup?
        self.thruster_map([100, 100, 100, 100, 100, 100, 100, 100])
        time.sleep(0.5)
        self.thruster_map(zero_thrust)

        signal.signal(signal.SIGINT, self.handler)

    def message_received(self, msg):  # called in subscription object initialization
        if self.check_change(msg):  # if the message has changed publish the new message
            publish = list(msg.thrusters)  # TODO: naming?
            print(publish)
            self.thruster_map(publish)

    def check_change(self, new_msg):
        new_thrust = list(new_msg.thrusters)

        # if value is unchanged, returns false to skip publish
        if self.prev_thrust == new_thrust:
            return False
        else:
            self.prev_thrust = new_thrust.copy()
            return True

    # TODO: naming?
    def thruster_map(self, thrust_list):
        thrusters = [0] * 8
        for i in range(8):
            thrusters[i] = 32767 + ((thrust_list[i] + 1) << 7)

        self.pca.channels[0].duty_cycle = thrusters[0]
        self.pca.channels[1].duty_cycle = thrusters[1]
        self.pca.channels[2].duty_cycle = thrusters[2]
        self.pca.channels[3].duty_cycle = thrusters[3]
        self.pca.channels[4].duty_cycle = thrusters[4]
        self.pca.channels[5].duty_cycle = thrusters[5]
        self.pca.channels[6].duty_cycle = thrusters[6]
        self.pca.channels[7].duty_cycle = thrusters[7]

    def handler(self, signum, frame):  # called when ctrl-C interrupt is detected
        print('Ctrl-C detected')
        # print(publish)
        self.thruster_map(zero_thrust)


def main(args=None):
    rclpy.init(args=args)
    node = ThrustToPWMNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
