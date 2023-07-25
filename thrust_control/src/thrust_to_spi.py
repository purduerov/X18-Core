#! /usr/bin/python3
import rclpy
import rclpy.node as Node
from spidev import SpiDev
from shared_msgs.msg import CanMsg, FinalThrustMsg, TempMsg
import signal

spi = SpiDev()
spi.open(0, 1) # bus,device

prev_thrust = [127,127,127,127,127,127,127,127]  # power of thrusters --> 127 is neutral
zero_thrust = [127,127,127,127,127,127,127,127]  # power of thrusters --> 127 is neutral

spi.max_speed_hz = 50000

def handler(signum, frame): #called when ctrl-C interrupt is detected
    global zero_thrust
    print('Ctrl-C detected')
    publish = bytearray(zero_thrust)
    #print(publish)
    spi.writebytes(publish)
    spi.close()

def message_received(msg): #called in subscription object initialization
    if checkChange(msg): #if the message has changed publish the new message
        publish = list(msg.thrusters)
        publish = bytearray(publish)
        #print(publish)
        spi.writebytes(publish)

def checkChange(new_msg):
    global prev_thrust
    new_thrust = list(new_msg.thrusters)
    #if value is unchanged, returns false to skip publish
    if prev_thrust == new_thrust:
        return False
    else:
        prev_thrust = new_thrust.copy()
        return True

if __name__ == "__main__":

    signal.signal(signal.SIGINT, handler)
    rclpy.init() #initializes ros communication

    node = rclpy.create_node('thrust_to_spi') #creates an instance of a node with name thrust_proc

    # Subscribe to final_thrust and start callback function
    sub = node.create_subscription(
                                    FinalThrustMsg, 
                                    'final_thrust',
                                    message_received, 
                                    10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
