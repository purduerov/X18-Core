#! /usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from spidev import SpiDev
from crccheck.crc import Crc32Mpeg2

import RPi.GPIO as GPIO

from shared_msgs.msg import FinalThrustMsg, ToolsMotorMsg


def invert_thrust(thrust_value):

    return 255 - thrust_value - 1


def split_bytes(num):
    byte2 = num & 0xFF
    byte1 = (num >> 8) & 0xFF
    return [byte1, byte2]


def swap_bytes(bytes_list):
    return [bytes_list[1], bytes_list[0]]


class ThrustToSPINode(Node):
    # initialize Class Variables
    ZERO_THRUST = [127, 127, 127, 127, 127, 127, 127, 127]  # 127 is neutral
    ZERO_TOOLS = [127, 127, 127, 127]  # WHAT IS THE NEUTRAL VALUE???
    FULL_THRUST_CONTROL = 2
    TOOLS_SERVO_CONTROL = 3
    identifier = 0
    blocked = False

    def __init__(self, bus=0, device=1, mode=0, speed=50000, bits_per_word=8):
        super().__init__("thrust_to_spi")

        # initialize logger
        logger = self.get_logger().info("INITIALIZED")

        # initialize SPI
        self.spi = SpiDev(bus, device)
        self.spi.mode = mode
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = bits_per_word

        # initialize values
        self.thrusters = self.ZERO_THRUST
        self.tools = self.ZERO_TOOLS
        self.pin = 8

        # initialize pull-down for chip select
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # GPIO.setup(24, GPIO.OUT, initial=GPIO.HIGH) # CE0,CE1 is 7,8; PINS 24 and 26

        # Subscribe to final_thrust and start callback function
        self.thrust_sub = self.create_subscription(
            FinalThrustMsg,  # message, updated 50 times per second regardless of change
            "final_thrust",  # topic
            self.thrust_received,  # callback function
            10,
        )

        self.tools_sub = self.create_subscription(
            ToolsMotorMsg, "/tools_motor", self.tools_received, 10
        )

        return

    def thrust_map(self, thrusters):
        mapped_thrusters = [127] * 8

        # mapped_thrusters[0] = invert_thrust(thrusters[6])
        # mapped_thrusters[1] = invert_thrust(thrusters[2])
        # mapped_thrusters[2] = thrusters[5]
        # mapped_thrusters[3] = thrusters[1]
        # mapped_thrusters[4] = invert_thrust(thrusters[7])
        # mapped_thrusters[5] = invert_thrust(thrusters[3])
        # mapped_thrusters[6] = (thrusters[0])
        # mapped_thrusters[7] = (thrusters[4])

        mapped_thrusters[0] = thrusters[6]
        mapped_thrusters[1] = thrusters[2]
        mapped_thrusters[2] = thrusters[5]
        mapped_thrusters[3] = thrusters[1]
        mapped_thrusters[4] = thrusters[7]
        mapped_thrusters[5] = thrusters[3]
        mapped_thrusters[6] = thrusters[0]
        mapped_thrusters[7] = thrusters[4]

        return mapped_thrusters

    # map thrusters to correct position and invert
    # def thrust_map(self, thrusters):
    #   mapped_thrusters = self.ZERO_THRUST
    #  mapped_thrusters[0] = invert_thrust(thrusters[6])
    # mapped_thrusters[1] = invert_thrust(thrusters[2])
    # mapped_thrusters[2] = thrusters[5]
    # mapped_thrusters[3] = thrusters[1]
    # mapped_thrusters[4] = invert_thrust(thrusters[7])
    # mapped_thrusters[5] = invert_thrust(thrusters[3])
    # mapped_thrusters[6] = thrusters[0]
    # mapped_thrusters[7] = thrusters[4]
    # return mapped_thrusters

    # thrust callback function
    def thrust_received(self, msg):
        print("THRUSTER RECEIVED")
        self.data = self.thrust_map(msg.thrusters)
        self.type = self.FULL_THRUST_CONTROL
        self.message_received()
        return

    def tools_received(self, msg):
        print("TOOLS RECEVIED")
        self.data = list(msg.tools)
        self.data += [0, 0, 0, 0]
        self.type = self.TOOLS_SERVO_CONTROL
        self.message_received()
        return

    # process a received message from subscription
    def message_received(self):
        if not self.blocked:
            self.set_message_id()
            self.transfer(self.format_message())
            self.response_handler()
        return

    # sets id to an incremented 2-byte number
    def set_message_id(self):
        if self.identifier == 65535:
            self.identifier = 0
        else:
            self.identifier += 1
        return

    # prepares each byte to be sent into list of bytes and gets CRC value
    def format_message(self):
        split_id = split_bytes(self.identifier)
        message = [self.type] + list(split_id) + list(self.data)
        self.compute_crc(message)
        split_crc = swap_bytes(split_bytes(self.crc))
        message += split_crc
        return bytearray(message)

    # prepares input and calls the Crc32Mpeg2 CRC function
    def compute_crc(self, message):
        message_list = []
        for item in message:
            message_list.extend([0x00, 0x00, 0x00, item])
        self.crc = Crc32Mpeg2.calc(message_list)
        return

    """
    message: 11 32-bit values
    input data format: words
    POLY = 0x4C11DB7
    """

    # sends the message and ~[pulls Chip Select pin low]
    def transfer(self, message):
        print("MASTER: ", list(message))
        if not self.blocked:
            self.last_message = list(message)
            # GPIO.output(self.pin, GPIO.LOW)
            self.spi.xfer3(message)
            # GPIO.output(self.pin, GPIO.HIGH)
        return

    # sends data to allow slace response
    def response_handler(self):
        if not self.blocked:
            time.sleep(0.0001)
            message = [0] * len(self.last_message)
            # GPIO.output(self.pin, GPIO.LOW)
            response = list(self.spi.xfer3(message))
            if (
                response[-2] != self.last_message[-2]
                or response[-1] != self.last_message[-1]
            ):
                print("ERROR: CRC VALUES DO NOT MATCH")
            # GPIO.output(self.pin, GPIO.HIGH)
            print("SLAVE: ", response)

        return

    # error and interrupt handler NEED TO UPDATE
    def handler(self):
        print("trl-C detected")
        self.blocked = True

        kill = [self.FULL_THRUST_CONTROL] + [0, 0] + self.ZERO_THRUST
        print(kill)
        self.compute_crc(kill)
        split_crc = swap_bytes(split_bytes(self.crc))
        kill += split_crc
        self.spi.xfer3(bytearray(kill))

        print("THRUST MASTER: ", list(kill))

        message = [self.TOOLS_SERVO_CONTROL] + [0, 0] + self.ZERO_TOOLS
        print(message)
        self.compute_crc(message)
        split_crc = swap_bytes(split_bytes(self.crc))
        message += split_crc
        self.spi.xfer3(bytearray(message))

        print("TOOLS MASTER: ", list(message))

        self.spi.close()
        # GPIO.cleanup()
        print("Closed")
        exit(1)


def main(args=None):
    # initialize node and ROS
    rclpy.init(args=args)
    node = ThrustToSPINode()

    GPIO.setup(22, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.output(22, GPIO.LOW)
    time.sleep(2)
    GPIO.output(22, GPIO.HIGH)

    # run node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handler()

    # cleanup
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
