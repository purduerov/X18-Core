#! /usr/bin/python3
from packets import ThrustPacket, ESCPacket, PowerPacket
import time
import rclpy
from rclpy.node import Node
from crccheck.crc import Crc32Mpeg2
import serial 
import RPi.GPIO as GPIO

from shared_msgs.msg import FinalThrustMsg

def split_bytes(num):
    byte2 = num & 0xFF
    byte1 = (num >> 8) & 0xFF
    return [byte1, byte2]

class ThrustToUARTNode(Node):
    def __init__(self):
        super().__init__("thrust_to_uart")

        logger = self.get_logger().info("UART INITIALIZED")

        self.identifier = 0

        self.THRUST_ID = 0x78

        # GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.OUT, initial=GPIO.LOW)

        self.ser = serial.Serial(
            port = '/dev/serial0',
            baudrate = 115200,
            timeout=0.1
        )

        if not self.ser.is_open:
            self.ser.open()

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

        mapped_thrusters[0] = thrusters[6]
        mapped_thrusters[1] = thrusters[2]
        mapped_thrusters[2] = thrusters[5]
        mapped_thrusters[3] = thrusters[1]
        mapped_thrusters[4] = thrusters[7]
        mapped_thrusters[5] = thrusters[3]
        mapped_thrusters[6] = thrusters[0]
        mapped_thrusters[7] = thrusters[4]

        return mapped_thrusters

    def thrust_received(self, msg):
        self.data = self.thrust_map(msg.thrusters)
        self.transfer(self.format_message())
        print(self.ser.read(12))

    def set_message_id(self):
        if self.identifier == 2**16:
            self.identifier = 1
        else:
            self.identifier += 1

    def transfer(self, msg):
        self.last_message = list(msg)
        print([hex(byte) for byte in self.last_message])
        #GPIO.output(23, GPIO.HIGH)
        two_bytes = self.ser.read(2)
        if not two_bytes:
            print("ERROR: NO DATA RECEIVED")
        self.ser.write(msg)
        # GPIO.output(23, GPIO.LOW)


    def response_handler(self):
        if not self.blocked:
            time.sleep(0.0001)

            # Read available bytes from UART (assuming termination by timeout or known format)
            response = self.ser.read(self.ser.in_waiting)  # Read all available bytes


            if not response:
                print("ERROR: No response received")
                return

            # Unpack the received response
            received_packet = BasePacket.unpack(response, data_length=len(response) - 4)

            if (received_packet.device_id != 1):
                print("ERROR: INVALID DEVICE ID")
                return
            # Unpack the last sent message
            sent_packet = BasePacket.unpack(self.last_message, data_length=len(self.last_message) - 4)

            # Compare CRC values
            if received_packet.crc != sent_packet.crc:
                print("ERROR: CRC VALUES DO NOT MATCH")

                # print("SLAVE: ", list(response))

        return

    
    def format_message(self):
        self.set_message_id()
        split_identifier = split_bytes(self.identifier)
        message_body = list(self.data)

        packet = ThrustPacket(device_id=self.THRUST_ID, message_id=self.identifier, data=self.data, crc=self.compute_crc(message_body))
        return packet.pack()

    def compute_crc(self, message):
        message_list = []
        for item in message:    
            message_list.extend([0x00, 0x00, 0x00, item])
        crc = Crc32Mpeg2.calc(message_list)
        print(hex(crc & 0xFF))
        return crc & 0xFF



    def handler(self):
        message_body = [self.THRUST_ID, 0, 0] + ([127] * 8)
        packet = ThrustPacket(device_id=self.THRUST_ID, message_id=0, data=([127] * 8), crc=self.compute_crc(message_body))
        self.ser.write(packet.pack())
        return

def main(args=None):
    rclpy.init(args=args)
    node = ThrustToUARTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handler()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()