#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import subprocess


class Camera(Node):
    def __init__(self):
        super().__init__("cameras")
        self.declare_parameter("dev_name", "device=/dev/video2")
        self.declare_parameter("port_num", "5600")

        dev_name = self.get_parameter("dev_name").get_parameter_value().string_value
        port = self.get_parameter("port_num").get_parameter_value().string_value

        while True:
            subprocess.run(
                [
                    "gst-launch-1.0",
                    "-v",
                    "v4l2src",
                    dev_name,
                    "!",
                    "video/x-h264,width=1920,height=1080",
                    "!",
                    "h264parse",
                    "!",
                    "queue",
                    "!",
                    "rtph264pay",
                    "config-interval=10",
                    "pt=96",
                    "!",
                    "udpsink",
                    "host=224.1.1.1",
                    "auto-multicast=true",
                    "port=" + port,
                    "sync=false",
                    "buffer-size=1048576",
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
