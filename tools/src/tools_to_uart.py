#! /usr/bin/python3
from packets import ToolsPacket
import rclpy
import time
from rclpy.node import Node
from crccheck.crc import Crc32Mpeg2
import serial 

from shared_msgs.msg import ToolsMotorMsg

def split_bytes(num):
    byte2 = num & 0xFF
    byte1 = (num >> 8) & 0xFF
    return [byte1, byte2]

class ToolsToUARTNode(Node):
    def __init__(self):
        super().__init__("tools_to_uart")

        logger = self.get_logger().info("UART INITIALIZED")

        self.identifier = 0

        self.TOOLS_ID = 2
        
        self.ser = serial.Serial(
            port = '/dev/serial0', # THIS WILL BE DIFFERENT
            baudrate = 9600,
            timeout=0.1
        )

        if not self.ser.is_open:
            self.ser.open()

        # Subscribe to tools motor and start callback function
        self.tools_sub = self.create_subscription(
            ToolsMotorMsg, 
            "/tools_motor", 
            self.tools_received, 
            10
        )

        return

    def tools_received(self, msg):
        self.data = msg.tools
        self.transfer(self.format_message())
        print(self.ser.read(12))

    def set_message_id(self):
        if self.identifier == 2**16:
            self.identifier = 1
        else:
            self.identifier += 1

    def transfer(self, msg):
        self.ser.write(msg)

    def format_message(self):
        self.set_message_id()
        split_identifier = split_bytes(self.identifier)
        message_body = [self.TOOLS_ID] + list(split_identifier) + list(self.data)

        packet = ToolsPacket(device_id=self.TOOLS_ID, message_id=self.identifier, data=self.data, crc=self.compute_crc(message_body))
        return packet.pack()

    def compute_crc(self, message):
        crc = Crc32Mpeg2.calc(message)
        print(hex(crc & 0xFF))
        return crc & 0xFF

    def handler(self):
        message_body = [self.TOOLS_ID, 0, 0] + ([127] * 7)
        packet = ToolsPacket(device_id=self.TOOLS_ID, message_id=0, data=([127] * 7), crc=self.compute_crc(message_body))
        self.ser.write(packet.pack())
        return

def main(args=None):
    rclpy.init(args=args)
    node = ToolsToUARTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handler()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()