#! /usr/bin/python3
import signal

import rclpy
from rclpy.node import Node
from spidev import SpiDev

from shared_msgs.msg import FinalThrustMsg

zero_thrust = [127, 127, 127, 127, 127, 127, 127, 127]  # power of thrusters --> 127 is neutral


class ThrustToSPINode(Node):
    prev_thrust = [127, 127, 127, 127, 127, 127, 127, 127]  # power of thrusters --> 127 is neutral

    def __init__(self):
        super().__init__('thrust_to_spi')

        self.spi = SpiDev()
        self.spi.open(0, 1)  # bus,device
        self.spi.max_speed_hz = 50000

        # Subscribe to final_thrust and start callback function
        self.sub = self.create_subscription(
            FinalThrustMsg,
            'final_thrust',
            self.message_received,
            10
        )

        signal.signal(signal.SIGINT, self.handler)

    def message_received(self, msg):  # called in subscription object initialization
        if self.check_change(msg):  # if the message has changed publish the new message
            publish = list(msg.thrusters)
            publish = bytearray(publish)

            self.spi.writebytes(publish)

    def check_change(self, new_msg):
        new_thrust = list(new_msg.thrusters)

        # if value is unchanged, returns false to skip publish
        if self.prev_thrust == new_thrust:
            return False
        else:
            self.prev_thrust = new_thrust.copy()
            return True

    def handler(self, signum, frame):  # called when ctrl-C interrupt is detected
        print('Ctrl-C detected')
        publish = bytearray(zero_thrust)
        # print(publish)
        self.spi.writebytes(publish)
        self.spi.close()


def main(args=None):
    rclpy.init(args=args)
    node = ThrustToSPINode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
