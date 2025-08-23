#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import subprocess


class Camera(Node):
    def __init__(self):
        super().__init__("cameras")
        self.declare_parameter("ip")
        self.declare_parameter("device", "device=/dev/video2")
        self.declare_parameter("camera_number", 1)

        ip_address = self.get_parameter("ip").get_parameter_value().string_value
        dev_name = self.get_parameter("device").get_parameter_value().string_value
        camera_num = (
            self.get_parameter("camera_number").get_parameter_value().integer_value
        )

        # Define RTSP stream URL
        rtsp_url = f"rtsp://{ip_address}:8554/camera_{camera_num}"

        try:
            # Run ffmpeg with correct argument order
            process = subprocess.run(
                [
                    "ffmpeg",
                    "-f",
                    "v4l2",
                    "-i",
                    dev_name,
                    "-fflags",
                    "nobuffer",
                    "-codec:v",
                    "copy",
                    "-g",
                    "10",
                    "-f",
                    "rtsp",
                    rtsp_url,
                ],
                check=True,
            )
            self.get_logger().info(f"Camera {camera_num} stream started")
        except subprocess.CalledProcessError:
            self.get_logger().error(f"Failed to start camera {camera_num}. Exiting.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
