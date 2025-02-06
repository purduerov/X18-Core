#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import subprocess


class Camera(Node):
    def __init__(self):
        super().__init__("cameras")
        self.declare_parameter("ip", "192.168.1.23")
        self.declare_parameter("device", "device=/dev/video2")

        ip_address = self.get_parameter("ip").get_parameter_value().string_value
        dev_name = self.get_parameter("device").get_parameter_value().string_value

        # Define RTSP stream URL 
        rtsp_url = "rtsp://{ip_address}:8554/{dev_name}"

        while True:
            subprocess.run(
                [
                    "ffmpeg", "-f", "v4l2", "--fflags", "nobuffer", 
                    "-i", dev_name, "-vcodec", "copy", "-g", "10",
                    "-f", "rtsp", rtsp_url
                ]
            )



def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
