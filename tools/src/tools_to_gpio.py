#! /usr/bin/python3

import rclpy
from shared_msgs.msg import ToolsCommandMsg
from rclpy.node import Node
import RPi.GPIO as GPIO
import signal 
import sys

gpio_pins = [16, 21, 23, 24, 20]

class Tools(Node):
    def __init__(self):
        super().__init__('solenoid_tools')
        GPIO.setmode(GPIO.BCM)
        for i in gpio_pins:
            print(i)
            GPIO.setup(i, GPIO.OUT, initial=GPIO.LOW)
        self.sub = self.create_subscription(ToolsCommandMsg,'/tools', self.message_received, 10)

    def message_received(self, msg):
        toolsarr = msg.tools
        uv_pin = gpio_pins[0]
        pm_pin = gpio_pins[3]
        ghost_pin = gpio_pins[1]
        secondary_pin = gpio_pins[2]
        fishfry_pin = gpio_pins[4]

        if(toolsarr[0]):
            GPIO.output(uv_pin, GPIO.HIGH)
            print("1 high")
        else:
            GPIO.output(uv_pin,GPIO.LOW)
            print("1 low")

        if(toolsarr[1]):
            GPIO.output(ghost_pin,True)
            print("2 high")
        else:
            GPIO.output(ghost_pin,GPIO.LOW)
            print("2 low")

        if(toolsarr[2]):
            GPIO.output(secondary_pin,True)
            print("3 high")
        else:
            GPIO.output(secondary_pin,GPIO.LOW)
            print("3 low")

        if(toolsarr[3]):
            GPIO.output(pm_pin,True)
            print("4 high")
        else:
            GPIO.output(pm_pin,GPIO.LOW)
            print("4 low")

        if(toolsarr[4]):
            GPIO.output(fishfry_pin,True)
            print("5 high")
        else:
            GPIO.output(fishfry_pin,GPIO.LOW)
            print("5 low")
        

def handle_exit(signum, frame):
    ("ctr-C detected")
    GPIO.cleanup()
    sys.exit(0)


def main(args=None):
    signal.signal(signal.SIGINT, handle_exit)
    rclpy.init(args = args)
    tools = Tools()
    rclpy.spin(tools)
    tools.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
