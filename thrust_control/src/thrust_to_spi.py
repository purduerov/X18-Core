#! /usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from spidev import SpiDev
from crccheck.crc import Crc32Mpeg2

import RPi.GPIO as GPIO

from shared_msgs.msg import FinalThrustMsg

def invert_thrust(thrust_value):
    return (255 - thrust_value - 1)

class ThrustToSPINode(Node):
    # initialize Class Variables
    ZERO_THRUST = [127, 127, 127, 127, 127, 127, 127, 127]  # 127 is neutral
    FULL_THRUST_CONTROL = 2
    identifier = 0
    blocked = False

    def __init__(self, bus=0, device=1, mode=0, speed=50000, bits_per_word=8):
        super().__init__('thrust_to_spi')

        # initialize logger
        logger = self.get_logger().info("INITIALIZED")

        # initialize SPI 
        self.spi = SpiDev(bus, device)
        self.spi.mode = mode
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = bits_per_word
        
        # initialize values
        self.thrusters = self.ZERO_THRUST
        self.pin = 8

        # initialize pull-down for chip select
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.HIGH)

        # Subscribe to final_thrust and start callback function
        self.sub = self.create_subscription(
            FinalThrustMsg, # message, updated 50 times per second regardless of change
            'final_thrust', # topic
            self.thrust_received, # callback function
            10
        )
        
        return

    # map thrusters to correct position and invert 
    def thrust_map(self, thrusters):
        mapped_thrusters = self.ZERO_THRUST
        mapped_thrusters[0] = invert_thrust(thrusters[6])
        mapped_thrusters[1] = invert_thrust(thrusters[2])
        mapped_thrusters[2] = thrusters[5]
        mapped_thrusters[3] = thrusters[1]
        mapped_thrusters[4] = invert_thrust(thrusters[7])
        mapped_thrusters[5] = invert_thrust(thrusters[3])
        mapped_thrusters[6] = thrusters[0]
        mapped_thrusters[7] = thrusters[4]
        return mapped_thrusters

    # thrust callback function
    def thrust_received(self, msg):
        print("THRUSTER RECEIVED")
        self.thrusters = self.thrust_map(msg.thrusters)
        self.message_received()
        return

    # process a received message from subscription
    def message_received(self):
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
   
    # swap bytes in a 2-byte list
    def swap_bytes(self, bytes_list):
        bytes_list = [bytes_list[1], bytes_list[0]]
        return bytes_list

    def format_message(self):
        split_id = self.split_bytes(self.identifier)
        message = list(self.type) + list(split_id) + list(self.thrusters)
        self.compute_crc(message)
        split_crc = self.split_bytes(self.crc)
        split_crc = self.swap_bytes(split_crc)
        message += split_crc
        formatted = bytearray(message)
        return formatted

    '''
    message: 11 32-bit values
    input data format: words
    POLY = 0x4C11DB7
    '''
    def compute_crc(self, message):
        message_list = []
        for item in message:
            message_list.extend([0x00, 0x00, 0x00, item])
        self.crc = Crc32Mpeg2.calc(message_list)
        return
    
    def transfer(self, message):
        print("MASTER: ", list(message))
        response = ()
        self.last_message = message
        if (not self.blocked):
            GPIO.output(self.pin, GPIO.LOW)
            response = list(self.spi.xfer3(bytearray(message)))
            GPIO.output(self.pin, GPIO.HIGH)
        return response
    
    def response_handler(self, response):
        if (not self.blocked):
            time.sleep(0.0001)
            message = [0] * 13
            GPIO.output(self.pin, GPIO.LOW)
            response = list(self.spi.xfer3(bytearray(message)))
            GPIO.output(self.pin, GPIO.HIGH)
            print("SLAVE: ", response)
        return 

    # error and interrupt handler 
    def handler(self):
        print('trl-C detected')
        self.blocked = True
        self.type = self.FULL_THRUST_CONTROL
        self.identifier = 0
        self.thrusters = self.ZERO_THRUST
        message = self.format_message()
        print("MASTER: ", list(message))
        GPIO.output(self.pin, GPIO.LOW)
        self.spi.xfer3(message)
        GPIO.output(self.pin, GPIO.HIGH)
        self.spi.close()
        GPIO.cleanup()
        print('Closed')
        exit(1)

def main(args=None):
    # initialize node and ROS
    rclpy.init(args=args)
    node = ThrustToSPINode()

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