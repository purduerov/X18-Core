#! /usr/bin/python3
import time
import lgpio as lg 
import rclpy
from rclpy.node import Node
from crccheck.crc import Crc32Mpeg2

# import RPi.GPIO as GPIO

from shared_msgs.msg import FinalThrustMsg, ToolsMotorMsg
from utils.heartbeat_helper import HeartbeatHelper

class ThrustToSPINode(Node):
    # initialize Class Variables
    ZERO_THRUST = [127, 127, 127, 127, 127, 127]  # 127 is neutral
    ZERO_TOOLS = [127, 127, 127, 127]  # WHAT IS THE NEUTRAL VALUE???
    FULL_THRUST_CONTROL = 2
    TOOLS_SERVO_CONTROL = 3
    identifier = 0
    blocked = False
    
    def __init__(self, device, channels : tuple, baud, flags):
        super().__init__("thrust_to_spi")
        # Setup heartbeat
        self.heartbeat_helper = HeartbeatHelper(self)

        # initialize logger
        logger = self.get_logger().info("INITIALIZED")
        # initialize values
        self.thrusters = self.ZERO_THRUST
        self.tools = self.ZERO_TOOLS

        self.thrust_handle = lg.spi_open(device, channels[0], baud, flags)
        self.tools_handle = lg.spi_open(device, channels[1], baud, flags)
        self.id = 0
        self.thrust_data = 6 * [0]
        self.tools_data = 6 * [0]
        self.blocked = False

        self.thrust_sub = self.create_subscription(
            FinalThrustMsg,  # message, updated 50 times per second regardless of change
            "final_thrust",  # topic
            self.thrust_received,  # callback function
            10,
        )

        self.thrust_response_pub = self.create_publisher(
            FinalThrustMsg,
            'thrust_response',
            10
        )
    
        self.tools_sub = self.create_subscription(
            ToolsMotorMsg, 'tools_motor', self.tools_received, 10
        )

        return


    def thrust_map(self, thrusters):
        mapped_thrusters = [127] * 6

        #NEED TO ADD CORRECT THRUSTER MAPPING
        mapped_thrusters[0] = thrusters[0]
        mapped_thrusters[1] = thrusters[1]
        mapped_thrusters[2] = thrusters[2]
        mapped_thrusters[3] = thrusters[3]
        mapped_thrusters[4] = thrusters[4]
        mapped_thrusters[5] = thrusters[5]

        return mapped_thrusters
    
    def tool_map(self, tools):
        mapped_tools = [127] * 6

        mapped_tools[0] = tools[0]
        mapped_tools[1] = tools[1]
        mapped_tools[2] = tools[2]
        mapped_tools[3] = tools[3]
        mapped_tools[4] = tools[4]
        mapped_tools[5] = tools[5]

        return mapped_tools


    # thrust callback function
    def thrust_received(self, msg):
        self.get_logger().info(f"THRUST SENT: {[hex(n) for n in self.thrust_data]}")
        self.thrust_data = self.thrust_map(msg.thrusters)
        self.message_received(self.thrust_data, 0xf, self.thrust_handle)
        return

    def tools_received(self, msg):
        self.get_logger().info(f"TOOLS SENT: {[hex(n) for n in self.tools_data]}")
        self.tools_data = self.tool_map(msg.tools)
        self.message_received(self.tools_data, 0xf, self.tools_handle)
        return

    # process a received message from subscription
    def message_received(self, data, msgType, handle):
        if not self.blocked:
            self.blocked = True
            received_mesage = self.transfer(self.format_message(data, msgType), handle)
            self.blocked = False
            self.response_handler(received_mesage)
        return

    # prepares input and calls the Crc32Mpeg2 CRC function
    def compute_crc(self, message):
        crc_int =int(Crc32Mpeg2.calc(message))
        self.crc = crc_int.to_bytes(4)
        return

    def transfer(self, data, handle):
        (count, rx_buf) = lg.spi_xfer(handle, data) #(count, rx_data
        if self.id == 0xF:
            self.id = 0x0
        else:
            self.id += 1
        self.get_logger().info(f"RECEIVED DATA {[hex(n) for n in list(rx_buf)]}") 
        return rx_buf #maybe return just the second part of the tuple

    def format_message(self, data, msgType):
        message = [msgType + (self.id << 4)] + list(data)
        self.compute_crc(message)
        message = bytearray(message)
        message += self.crc
        return message
    

    # sends data to allow slave response
    def response_handler(self, response : bytearray):
        response_data = response[:-4]
        response_crc = int.from_bytes(response[-4:], byteorder="big")

        calc_crc = Crc32Mpeg2.calc(response_data)
       
        if(calc_crc != response_crc):
            print('WRONG CRC')
            return
        else:
            msg_id = (response_data[0] >> 4) & 0x0F
            msg_type = response_data[0] & 0x0F
            response_data = response_data[1:]
            msg = FinalThrustMsg()
            msg.thrusters = list(response_data)
            self.thrust_response_pub.publish(msg)
            

    

    def handler(self):
        print("Ctrl-C detected")
        self.blocked = True

        kill_msg = self.format_message(6 * [0], 0x0)
        self.transfer(kill_msg, self.thrust_handle)

        lg.spi_close(self.thrust_handle)
        lg.spi_close(self.power_handle)
        print("Closed")
        exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = ThrustToSPINode(0, (0, 1), 10000, 0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handler
   

if __name__ == "__main__":
    main()

        

