#! /usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from spidev import SpiDev
from crccheck.crc import Crc32Mpeg2

import RPi.GPIO as GPIO

from shared_msgs.msg import FinalThrustMsg

class ThrustToSPINode(Node):
    ZERO_THRUST = [127, 127, 127, 127, 127, 127, 127, 127]  # power of thrusters --> 127 is neutral
    FULL_THRUST_CONTROL = 2
    identifier = 0
    blocked = False

    def __init__(self, bus=0, device=1, mode=0, speed=50000, bits_per_word=8):
        super().__init__('thrust_to_spi')

        logger = self.get_logger().info("INITIALIZED")

        self.spi = SpiDev(bus, device)
        self.spi.mode = mode
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = bits_per_word
        
        self.thrusters = [127] * 8

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(24, GPIO.OUT, initial=GPIO.HIGH)

        # Subscribe to final_thrust and start callback function
        self.sub = self.create_subscription(
            FinalThrustMsg, # updated 50 times per second, called each time regardless of no change
            'final_thrust',
            self.thrust_received,
            10
        )
        
        return

    def thrust_received(self, msg):
        print("THRUSTER RECEIVED")
        self.thrusters = msg.thrusters
        self.message_received()
        return

    def message_received(self):  # called in subscription object initialization
        if (not self.blocked):    
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
    
    # only first bytes of num are returned
    def split_bytes(self, num):
        byte2 = num & 0xFF
        byte1 = (num >> 8) & 0xFF
        return [byte1, byte2]
   
    def swap_bytes(self, bytes_list):
        bytes_list = [bytes_list[1], bytes_list[0]]
        return bytes_list

    def format_message(self):
        split_id = self.split_bytes(self.identifier)
        type_list = [self.type]
        message = list(type_list) + list(split_id) + list(self.thrusters)
        self.compute_crc(message)
        split_crc = self.split_bytes(self.crc)
        split_crc = self.swap_bytes(split_crc)
        message += split_crc
        formatted = bytearray(message)
        return formatted

    def compute_crc(self, message):
        message_list = []
        for item in message:
            message_list.extend([0x00, 0x00, 0x00, item])
        self.crc = Crc32Mpeg2.calc(message_list)
        return
    '''
    message: 11 32-bit values
    input data format: words
    POLY = 0x4C11DB7
    '''
    
    def transfer(self, message):
        print("MASTER: ", list(message))
        response = ()
        self.last_message = message
        if (not self.blocked):
            GPIO.output(24, GPIO.LOW)
            response = self.spi.xfer3(message)
            GPIO.output(24, GPIO.HIGH)
        return response
    
    def response_handler(self, response):
        print("1st SLAVE: ", response)
        if (not self.blocked):
            time.sleep(0.0001)
            message = [0] * 13
            GPIO.output(24, GPIO.LOW)
            response = self.spi.xfer3(bytearray(message))
            GPIO.output(24, GPIO.HIGH)
            print("2nd SLAVE: ", response)
        return 

    def handler(self):  # called when ctrl-C interrupt is detected
        print('trl-C detected')
        self.blocked = True
        self.type = self.FULL_THRUST_CONTROL
        self.identifier = 0
        self.thrusters = self.ZERO_THRUST
        message = self.format_message()
        print("MASTER: ", list(message))
        self.spi.xfer3(message)
        self.spi.close()
        GPIO.cleanup()
        print('Closed')
        exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = ThrustToSPINode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handler()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()