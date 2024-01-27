#! /usr/bin/python3
import signal

import rclpy
from rclpy.node import Node
from spidev import SpiDev

from shared_msgs.msg import FinalThrustMsg

# for testing
from random import randint


class ThrustToSPINode(Node):
    ZERO_THRUST = [127, 127, 127, 127, 127, 127, 127, 127]  # power of thrusters --> 127 is neutral
    FULL_THRUST_CONTROL = 2
    identifier = 0
    
    def __init__(self, bus=0, device=1, mode=0, speed=50000, bits_per_word=8):
        super().__init__('thrust_to_spi')

        logger = self.get_logger().info("INITIALIZED")

        self.spi = SpiDev(bus, device)
        self.spi.mode = mode
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = bits_per_word

        # Subscribe to final_thrust and start callback function
        self.sub = self.create_subscription(
            FinalThrustMsg, # updated 50 times per second, called each time regardless of no change
            'final_thrust',
            self.message_received,
            10
        )

        signal.signal(signal.SIGINT, self.handler)

    def message_received(self, msg):  # called in subscription object initialization
        print("RECEIVED")
        self.type = self.FULL_THRUST_CONTROL
        self.set_message_id()
        self.thrusters = msg.thrusters # msg is a class containing thruster numpy array
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
    
    def format_message(self):
        split_id = self.split_bytes(self.identifier)
        type_list = [self.type]
        message = list(type_list) + list(split_id) + list(self.thrusters) 
        self.compute_crc() # TODO: figure out how to compute CRC with polynomial and passed parameters
        split_crc = self.split_bytes(self.crc)
        message += split_crc
        formatted = bytearray(message)
        return formatted

    # TODO: MUST COMPLETE CRC CALCULATIONS, REPLACE CODE IN FUNCTION
    def compute_crc(self):
        self.crc = randint(0, 65535)
        return
    
    def transfer(self, message):
        print("MASTER: ", list(message))
        response = self.spi.xfer3(message)
        return response
    
    # TODO: HOW TO USE THIS???
    def response_handler(self, response):
        print("SLAVE: ", response)
        return 

    # TODO: MESSAGES CAN STILL BE SENT AFTER HANDLER IS CALLED, FIGURE HOW TO PREVENT SENDING OF MESSAGES
    def handler(self, signum, frame):  # called when ctrl-C interrupt is detected
        print('Ctrl-C detected')
        '''
        publish = bytearray(zero_thrust)
        # print(publish)
        self.spi.writebytes(publish)
        self.spi.close()
        '''
        self.type = self.FULL_THRUST_CONTROL
        self.identifier = 0
        self.thrusters = self.ZERO_THRUST
        response = self.transfer(self.format_message())
        self.spi.close()


def main(args=None):
    rclpy.init(args=args)
    node = ThrustToSPINode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



""" 
import signal # Interrupt

import rclpy # ROS Node
import rclpy.node as Node # ROS Node
from spidev import SpiDev # SPI library

from shared_msgs.msg import FinalThrustMsg # Thruster Data Input

# JUST FOR TESTING PURPOSES
import time
# NEEDED TO CHANGE THIS
from MY_CRC import crc_check, crc_remainder # CRC funcs

# Constants
FULL_THRUST_CONTROL = 2
TOOLS_THRUST_CONTROL = 3
ACKNOWLEDGE_BYTES = 5

# Global Variables
off_thrust = [127,127,127,127,127,127,127,127]
data_val = 200
identifier  = 0

class SPI_message():
    def __init__(self, bus=0, device=1, mode=0, speed=500000, bits_per_word=8):
        self.spi = SpiDev(bus, device)
        self.spi.mode = mode
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = bits_per_word

    def format_data(self, data):
        formatted = bytearray(data)
        return formatted

    def transfer(self, data):
        response = self.spi.xfer3(data) 
        return response
    
    def interrupt(self, data):
        self.spi.xfer3(data)
        self.close_SPI()

    def message_ID(self):
        global identifier
        if identifier == 65535:
            identifier = 0
            return identifier
        identifier += 1
        return identifier
    
    def CRC(self):
        self.crc = 0

    def close_SPI(self):
        self.spi.close()

def send_and_receive(SPI_message):
    SPI_message.transfer()


if __name__ == "__main__":
    new_message = SPI_message()                         # initialize a message class
    signal.signal(signal.SIGINT, new_message.interrupt) # setup custom handler for interrupt
    rclpy.init()                                        # initialize ROS

    node =  rclpy.create_node('SPI_class')  # create a node called SPI_class
    sub = node.create_subscription(
        FinalThrustMsg,                     # subscribe to the message FinalThrustMsg
        'final_thrust',                     # create a topic called 'final_thrust'
        send_and_receive(new_message),      # call send_and_receive whenever FinalThrustMsg is called
        10                                  # set the Quality of Service queue to 10
    )

    rclpy.spin(node)        # continually runs the subscription and callback function
    node.destroy_node()     # stops the node
    rclpy.shutdown()        # stops ROS 
"""
