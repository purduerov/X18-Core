#!/usr/bin/env python3

# Import necessary libraries
import rclpy
import ipaddress
import subprocess
from rclpy.node import Node
from std_msgs.msg import String
import threading


class IpSubscriberNode(Node):
    def __init__(self):
        super().__init__('ip_subscriber_node')
        
        self.create_subscription(String, 'surface_ip', self.get_ip, 10)

        self.cameras_launched = False

    def get_ip(self, msg):
        received_ip = msg.data
        if self.cameras_launched:
            return

        try:
            ipaddress.ip_address(received_ip) 

            self.get_logger().info(f'Received from surface_ip topic: "{msg.data}"')

            ## Subprocess command
            self.launch_camera(received_ip)

        except ValueError:
            self.get_logger().info(f'Invalid IP received from topic: "{msg.data}"')

    def launch_camera(self, ip):
        self.get_logger().info(f"Launching camera with IP: {ip}")
        # Run command: v4l2-ctl --list-devices
        output = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True).stdout
        self.get_logger().info(f"Output of v4l2-ctl --list-devices: {output}")
        lines = output.splitlines()
        explorehd_devices = []
        i = 0
        while i < len(lines):
            if "exploreHD" in lines[i]:
                devices = []
                i += 1
                while i < len(lines) and lines[i].startswith("\t"):
                    devices.append(lines[i].strip())
                    i += 1
                if len(devices) >= 3:
                    explorehd_devices.append(devices[2])  # Third device (0-based index)
            else:
                i += 1

        self.get_logger().info(f"Discovered devices: {explorehd_devices}")
        
        # Handle the case where no devices are found
        if len(explorehd_devices) == 0:
            self.get_logger().error("No exploreHD devices found.")
            return

        # Launch nodes with the discovered devices
        i = 1
        for device in explorehd_devices:
            if i > 4:
                self.get_logger().info("Device limit reached, not launching more nodes.")
                break
            else:
                self.get_logger().info(f"Launching node with device: {device}, to camera number: {i}")
                cmd = [
                    "ros2", "run", "videos", "videos_launch.py", "--ros-args",
                    "-p", f"ip:={ip}",
                    "-p", f"device:={device}",
                    "-p", f"camera_number:={i}"
                ]
                thread = threading.Thread(target=subprocess.run, args=(cmd,), kwargs={"check": True})
                thread.start()
                i += 1

def main():
    rclpy.init()
    node = IpSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()