import rclpy # ROS Node
import rclpy.node as Node # ROS Node
import spidev # SPI library
import signal # Interrupt
from shared_msgs.msg import FinalThrustMsg # Thruster Data Input

# JUST FOR TESTING PURPOSES
import time
# NEEDED TO CHANGE THIS
from test_crc import crc_check, crc_remainder # CRC funcs

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
        self.spi = spidev.SpiDev(bus, device)
        self.spi.mode = mode
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = bits_per_word

    def format_data(self, data):
        formatted = bytearray(data)
        return formatted

    def transfer(self, data):
        response = self.spi.xfer3(data) 
        return response
    
    # how should data be passed?
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

    def readSlave(self):
        response = self.spi.readbytes(ACKNOWLEDGE_BYTES)
        return response
    
    def close_SPI(self):
        self.spi.close()

if __name__ == "__main__":
    new_message = SPI_message()
    signal.signal(signal.SIGINT, new_message.interrupt) # setup custom handler for interrupt
    """
    rclpy.init()

    node =  rclpy.create_node('SPI_class')
    sub = node.create_subscription(
        FinalThrustMsg,
        'final_thrust',
        # message_received,
        # 10
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 
    """

    while True:
        # set thruster array
        thrusters = 8 * [data_val]

        # set values
        types = 2
        message_id = 35677

        # create bytearray
        message = bytearray()
        message.extend(types.to_bytes(1, 'little'))
        message.extend(message_id.to_bytes(2, 'little'))
        message.extend(thrusters)

        # create integer
        message_int = int.from_bytes(message, byteorder='big')

        # get polynomial in binary
        polynomial = bin(79764919)
        polynomial = polynomial.lstrip('0b')

        # get message_int in binary
        message_bin = bin(message_int)
        message_bin = message_bin.lstrip('0b')

        # calculate crc remainder and get last 16 bits
        array = crc_remainder(message_bin, polynomial, '0')
        result = array[-16:]
        
        # convert remainder into integer
        crc = int(result, 2)

        # Add crc to end of message
        message.extend(crc.to_bytes(2, 'little'))

        response = new_message.transfer(message)
        print(response)

        # wait 350 ns
        time_end = time.time_ns() + 100000
        while time.time_ns() < time_end:
            counter = 0
            
        response = new_message.transfer(message)
        print(response)

        # wait 2 seconds
        time_end = time.time() + 2
        while time.time() < time_end:
            counter = 0