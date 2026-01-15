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
    
    def __init__(self, device, channel, baud, flags):
        super().__init__("thrust_to_spi")
        # Setup heartbeat
        self.heartbeat_helper = HeartbeatHelper(self)

        # initialize logger
        logger = self.get_logger().info("INITIALIZED")
        # initialize values
        self.thrusters = self.ZERO_THRUST
        self.tools = self.ZERO_TOOLS

<<<<<<< HEAD
        self.handle = lg.spi_open(device, channel, baud, flags)
        self.id = 0
=======
        self.spi_handle = lg.spi_open(device, channel, baud, flags)
        self.id = 0
        self.data = 6 * [0]
>>>>>>> cfbe5a8 (Innovative CRC working)

        self.thrust_sub = self.create_subscription(
            FinalThrustMsg,  # message, updated 50 times per second regardless of change
            "final_thrust",  # topic
            self.thrust_received,  # callback function
            10,
        )

        #EDIT LATER
        # self.tools_sub = self.create_subscription(
        #     ToolsMotorMsg, "/tools_motor", self.tools_received, 10
        # )

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


    # thrust callback function
    def thrust_received(self, msg):
        self.get_logger().info(f"THRUST RECEIVED: {[hex(n) for n in self.data]}")
        self.data = self.thrust_map(msg.thrusters)
        self.type = self.FULL_THRUST_CONTROL
        self.message_received()
        return

    def tools_received(self, msg):
        self.get_logger().info("TOLLDS")
        self.data = list(msg.tools)
        self.data += [0, 0, 0, 0]
        self.type = self.TOOLS_SERVO_CONTROL
        self.message_received()
        return

    # process a received message from subscription
    def message_received(self):
        if not self.blocked:
            self.transfer(self.format_message(self.data, 0xf))
            #self.response_handler()
        return

<<<<<<< HEAD
    # sets id to an incremented 2-byte number
    def set_message_id(self):
        if self.identifier == 65535:
            self.identifier = 0
        else:
            self.identifier += 1
        return

=======
>>>>>>> cfbe5a8 (Innovative CRC working)
    # prepares input and calls the Crc32Mpeg2 CRC function
    def compute_crc(self, message):
        crc_int =int(Crc32Mpeg2.calc(message))
        self.crc = crc_int.to_bytes(4)
        return

    def transfer(self, data):
<<<<<<< HEAD
        rx_buf = lg.spi_xfer(self.handle, data) #(count, rx_data)
        
=======
        (count, rx_buf) = lg.spi_xfer(self.spi_handle, data) #(count, rx_data)
>>>>>>> cfbe5a8 (Innovative CRC working)
        if self.id == 15:
            self.id = 0
        else:
            self.id += 1
<<<<<<< HEAD
        
=======
        self.get_logger().info(f"RECEIVED DATA {[hex(n) for n in list(rx_buf)]}") 
>>>>>>> cfbe5a8 (Innovative CRC working)
        return rx_buf #maybe return just the seconnd part of the tuple

    def format_message(self, data, msgType):
        message = [msgType + (self.id << 4)] + list(data)
<<<<<<< HEAD
        return bytearray(message)
=======
        self.compute_crc(message)
        message = bytearray(message)
        message += self.crc
        return message
>>>>>>> cfbe5a8 (Innovative CRC working)
    

    # sends data to allow slave response
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
    rclpy.init(args=args)
    node = ThrustToSPINode(0, 0, 10000, 0)

    try:
        rclpy.spin(node)
<<<<<<< HEAD
    except KeyboardInterrupt
=======
    except KeyboardInterrupt:
>>>>>>> cfbe5a8 (Innovative CRC working)
        node.handler
   
   
    # spiDevice = spiWork(0, 0, 10000, 0)
    # thrustValues = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]
    # formattedMessage = spiDevice.format_message(thrustValues, 0x1)
    # print(list(formattedMessage))
    # actualData = spiDevice.transfer(formattedMessage)
    # print(actualData)

if __name__ == "__main__":
    main()

        

