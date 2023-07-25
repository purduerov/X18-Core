#! /usr/bin/python3
import rclpy
import rclpy.node as Node
import board
import signal
import time
from shared_msgs.msg import FinalThrustMsg
import busio
import adafruit_pca9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

pca.frequency = 500

prev_thrust = [127,127,127,127,127,127,127,127]  # power of thrusters --> 127 is neutral
zero_thrust = [127,127,127,127,127,127,127,127]  # power of thrusters --> 127 is neutral

def thruster_map(thrustList):
    thrusters  = [0] * 8
    for i in range(8):
        thrusters[i] = 32767 + ((thrustList[i] + 1) << 7)

    pca.channels[0].duty_cycle = thrusters[0]
    pca.channels[1].duty_cycle = thrusters[1]
    pca.channels[2].duty_cycle = thrusters[2]
    pca.channels[3].duty_cycle = thrusters[3]
    pca.channels[4].duty_cycle = thrusters[4]
    pca.channels[5].duty_cycle = thrusters[5]
    pca.channels[6].duty_cycle = thrusters[6]
    pca.channels[7].duty_cycle = thrusters[7]



def handler(signum, frame): #called when ctrl-C interrupt is detected
    global zero_thrust
    print('Ctrl-C detected')
  #  print(publish)
    thruster_map(zero_thrust)

def message_received(msg): #called in subscription object initialization
    if checkChange(msg): #if the message has changed publish the new message
        publish = list(msg.thrusters)
        print(publish)
        thruster_map(publish)

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

    thruster_map([100, 100, 100, 100, 100, 100, 100, 100])

    time.sleep(0.5)

    thruster_map(zero_thrust)

    signal.signal(signal.SIGINT, handler)
    rclpy.init() #initializes ros communication

    node = rclpy.create_node('thrust_to_pwm') #creates an instance of a node with name thrust_proc

    # Subscribe to final_thrust and start callback function
    sub = node.create_subscription(
                                    FinalThrustMsg, 
                                    'final_thrust',
                                    message_received, 
                                    10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
