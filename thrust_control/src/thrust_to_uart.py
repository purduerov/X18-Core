#! /usr/bin/python3
from packets import ThrustPacket, BasePacket, ESCPacket, PowerPacket
import time
import rclpy
from rclpy.node import Node
from crccheck.crc import Crc32Mpeg2
import serial 
import wiringpi
import RPi.GPIO as GPIO
import struct

from shared_msgs.msg import FinalThrustMsg
from utils.heartbeat_helper import HeartbeatHelper

def split_bytes(num):
    byte2 = num & 0xFF
    byte1 = (num >> 8) & 0xFF
    return [byte1, byte2]

class ThrustToUARTNode(Node):
    def __init__(self):
        super().__init__("thrust_to_uart")

        logger = self.get_logger().info("UART INITIALIZED")

        # Heartbeat
        self.heartbeat_helper = HeartbeatHelper(self)

        # set initial values
        self.identifier = 0
        self.THRUST_ID = 0x78
        self.blocked = False

        # setup hardware configurations
        wiringpi.wiringPiSetup()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)

        # open UART serial port 
        # sudo chmod 666 /dev/serial0
        self.ser = serial.Serial(
            port = '/dev/serial0',
            baudrate = 115200,
            timeout=0.1
        )

        if not self.ser.is_open:
            self.ser.open()

        # wait to fill buffer in STM
        """
        fill_buffer_byte = [255]
        while True:
            GPIO.output(18, GPIO.HIGH)
            self.ser.write(struct.pack(">1B", *fill_buffer_byte))
            self.get_logger().info(str(fill_buffer_byte))
            self.ser.flush()
            GPIO.output(18, GPIO.LOW)
            response = self.ser.read()
            if response:
                self.get_logger().info(response)
                break
            time.sleep(1)
        """

        # Subscribe to final_thrust and start callback function
        self.thrust_sub = self.create_subscription(
            FinalThrustMsg,  # message, updated 50 times per second regardless of change
            "final_thrust",  # topic
            self.thrust_received,  # callback function
            10
        )

        return

    def thrust_map(self, thrusters):
        mapped_thrusters = [127] * 8
        mapped_thrusters[0] = thrusters[5]
        mapped_thrusters[1] = thrusters[1]
        mapped_thrusters[2] = thrusters[2]
        mapped_thrusters[3] = thrusters[6]
        mapped_thrusters[4] = thrusters[4]
        mapped_thrusters[5] = thrusters[0]
        mapped_thrusters[6] = thrusters[3]
        mapped_thrusters[7] = thrusters[7]
        return mapped_thrusters

    def thrust_received(self, msg):
        self.data = self.thrust_map(msg.thrusters)
        self.transfer(self.format_message())
        self.response_handler()
        return

    def set_message_id(self):
        if self.identifier == 2**16:
            self.identifier = 1
        else:
            self.identifier += 1
        return

    def transfer(self, msg):
        self.last_message = list(msg)
        if not self.ser.in_waiting and not self.blocked:
            GPIO.output(18, GPIO.HIGH)
            self.ser.write(msg)
            #self.ser.flush()
            wiringpi.delayMicroseconds(950) # self.ser.write(msg) only fills up the buffers, but doesn't wait for msg to second, need a delay before pin is pulled low
            GPIO.output(18, GPIO.LOW)
        return

    def response_handler(self):
        if not self.blocked:
            time.sleep(0.001)

            # Read available bytes from UART (assuming termination by timeout or known format)
            response = self.ser.read()  # Read all available bytes

            if not response:
                # self.get_logger().info("ERROR: No response received")
                return

            self.get_logger().info("RECEIVED: " + str(list(response)))

            try:
                received_packet = BasePacket.unpack(response, data_length=len(response) - 4)
                if received_packet.device_id != 1: 
                    self.get_logger().info("ERROR: INVALID DEVICE ID")
                sent_packet = BasePacket.unpack(self.last_message, data_length=len(self.last_message) - 4)
                if received_packet.crc != sent_packet.crc:
                    self.get_logger().info("ERROR: CRC VALUES DO NOT MATCH")
            except:
                self.get_logger().info("Unable to unpack received packet")

        return
    
    def format_message(self):
        self.set_message_id()
        message_body = list(self.data)
        packet = ThrustPacket(device_id=self.THRUST_ID, message_id=self.identifier, data=self.data, crc=self.compute_crc(message_body))
        self.get_logger().info(f"Sending: {list(packet.pack())}")
        return packet.pack()

    def compute_crc(self, message):
        message_list = []
        for item in message:    
            message_list.extend([0x00, 0x00, 0x00, item]) # processed as 32-bit values on STM, must add padding to create 32-bit number
        crc = Crc32Mpeg2.calc(message_list)
        return crc & 0xFF

    def handler(self):
        self.get_logger().info("\nKeyboardInterrupt recieved: Stopping node...")
        self.blocked = True
        message_body = [self.THRUST_ID, 0, 0] + ([127] * 8)
        crc_val = self.compute_crc(message_body)
        packet = ThrustPacket(device_id=self.THRUST_ID, message_id=0, data=([127] * 8), crc=crc_val)
        self.ser.write(packet.pack())
        self.get_logger().info(" ".join(f"{x:02x}" for x in (message_body + [crc_val])))
        GPIO.cleanup() # remove GPIO configuration
        return

def main(args=None):
    rclpy.init(args=args)
    # time.sleep(10)
    node = ThrustToUARTNode()

    # GPIO.setwarnings(False)
    # GPIO.setmode(GPIO.BCM)
    # GPIO.setup(5, GPIO.OUT, initial=GPIO.HIGH)

    # GPIO.output(5, GPIO.LOW)
    # time.sleep(1)
    # GPIO.output(5, GPIO.HIGH)


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handler()
    
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()