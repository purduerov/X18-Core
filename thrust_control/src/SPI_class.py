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



# look at pi_temp under X16-Core/sensors/src