#! /usr/bin/python3

import time
import rclpy
from rclpy.node import Node
from spidev import SpiDev
from crccheck.crc import Crc32Mpeg2

import RPi.GPIO as GPIO

from shared_msgs.msg import ToolsCommandMsg


class ToolsToSPINode(Node):
    # initialize class globals
    ZERO_TOOLS = [127, 127, 127, 127]  # power of thrusters where 127 is neutral
    FULL_THRUST_CONTROL = 2
    identifier = 0
    blocked = False

    def __init__(self, bus=0, device=1, mode=0, speed=50000, bits_per_word=8):
        super().__init__("tools_to_spi")

        # initialize logger
        logger = self.get_logger().info("INITIALIZED")

        # initialize SPI
        self.spi = SpiDev(bus, device)
        self.spi.mode = mode
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = bits_per_word

        # initialize values
        self.tools = self.ZERO_TOOLS
        self.pin = 8

        # initialize pull-down for Chip Select
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.HIGH)

        # Subscribe to topics and start callback function
        self.sub = self.create_subscription(
            ToolsCommandMsg,  # message
            "tools_control",  # topic
            self.tools_received,  # callback function
            10,
        )

        return

    # tools callback function
    def tools_received(self, msg):
        print("TOOLS RECEIVED")
        self.tools = msg.tools
        self.message_received()
        return

    # update values and send message when new message is received
    def message_received(self):
        if not self.blocked:
            self.type = self.FULL_THRUST_CONTROL
            self.set_message_id()
            response = self.transfer(self.format_message())
            self.response_handler(response)
        return

    # sets id to an incremented 2-byte number
    def set_message_id(self):
        if self.identifier == 65535:
            self.identifier = 0
        else:
            self.identifier += 1
        return

    # split integer into bytes, returns only first two
    def split_bytes(self, num):
        byte2 = num & 0xFF
        byte1 = (num >> 8) & 0xFF
        return [byte1, byte2]

    # swap the first and second bytes in a two-byte list
    def swap_bytes(self, bytes_list):
        bytes_list = [bytes_list[1], bytes_list[0]]
        return bytes_list

    # prepares each byte to be sent into list of bytes and gets CRC value
    def format_message(self):
        split_id = self.split_bytes(self.identifier)
        message = [self.type] + list(split_id) + list(self.tools)
        self.compute_crc(message)
        split_crc = self.split_bytes(self.crc)
        split_crc = self.swap_bytes(split_crc)
        message += split_crc
        self.prev_message = message
        formatted = bytearray(message)
        return formatted

    # prepares input and calls the Crc32Mpeg2 CRC function
    def compute_crc(self, message):
        message_list = []
        for item in message:
            message_list.extend([0x00, 0x00, 0x00, item])
        self.crc = Crc32Mpeg2.calc(message_list)
        return

    """
    message: 15 32-bit values
    input data format: words
    polynomial = 0x4C11DB7
    """

    # sends the message and ~[pulls Chip Select pin low]
    def transfer(self, message):
        print("MASTER: ", list(message))
        if not self.blocked:
            # GPIO.output(self.pin, GPIO.LOW)
            response = self.spi.xfer3(message)
            # GPIO.output(self.pin, GPIO.HIGH)
        return response

    # sends data to allow slave response
    def response_handler(self, response):
        if not self.blocked:
            time.sleep(0.0001)
            message = [0] * 9
            # GPIO.output(self.pin, GPIO.LOW)
            response = list(self.spi.xfer3(bytearray(message)))
            if (
                response[7] != self.prev_message[7]
                or response[8] != self.prev_message[8]
            ):
                print("ERROR: CRC VALUES DO NOT MATCH")
            # GPIO.output(self.pin, GPIO.HIGH)
            print("SLAVE: ", response)
        return

    # called when Ctrl-C is press, sends stop thrusters and closes ports
    def handler(self):
        print("trl-C detected")
        self.blocked = True
        self.type = self.FULL_THRUST_CONTROL
        self.identifier = 0
        self.tools = self.ZERO_TOOLS
        message = self.format_message()
        print("MASTER: ", list(message))
        # GPIO.output(self.pin, GPIO.LOW)
        self.spi.xfer3(message)
        # GPIO.output(self.pin, GPIO.HIGH)
        self.spi.close()
        GPIO.cleanup()
        print("Closed")
        exit(1)


def main(args=None):
    # initialize node and ROS
    rclpy.init(args=args)
    node = ToolsToSPINode()

    # activate reset pin
    GPIO.setup(23, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.output(23, GPIO.LOW)
    time.sleep(2)
    GPIO.output(23, GPIO.HIGH)

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
