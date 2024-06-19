#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import subprocess


class PiTemp(Node):
    def __init__(self):
        super().__init__('cameras')
        subprocess.run("gst-launch-1.0 -v v4l2src device=/dev/video2 ! video/x-h264, width=1920,height=1080! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! udpsink host=224.1.1.1 auto-multicast=true port=5600 sync=false buffer-size=1048576")
        print("got here")
    
    def timer_callback(self):
        subprocess.run("gst-launch-1.0 -v v4l2src device=/dev/video2 ! video/x-h264, width=1920,height=1080! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! udpsink host=224.1.1.1 auto-multicast=true port=5600 sync=false buffer-size=1048576")



def main(args=None):
    rclpy.init(args = args)
    pi_temp = PiTemp()
    rclpy.spin(pi_temp)
    pi_temp.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
