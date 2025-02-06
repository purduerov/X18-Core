#!/usr/bin/env python3

# Import necessary libraries
import rclpy
import ipaddress
import subprocess
from rclpy.node import Node
from std_msgs.msg import String
import threading
from utils.heartbeat_helper import HeartbeatHelper


class IpSubscriberNode(Node):
    def __init__(self):
        super().__init__('ip_subscriber_node')

        # Set up heartbeat
        self.heartbeat_helper = HeartbeatHelper(self)
        
        self.create_subscription(String, 'surface_ip', self.get_ip, 10)
        # self.launch_camera("192.168.1.23")
        

    def get_ip(self, msg):
        received_ip = msg.data

        try:
            ipaddress.ip_address(received_ip) 
            
            self.publisher = self.create_publisher(String, 'surface_ip', 10)
            self.timer = self.create_timer(1.0, self.publish_stop)
            self.stop_count = 0
            self.stop_max_count = 5
            self.get_logger().info(f'Received from surface_ip topic: "{msg.data}"')

            ## Subprocess command
            self.launch_camera(received_ip)

        except ValueError:
            self.get_logger().info(f'Invalid IP received from topic: "{msg.data}"')

    def launch_camera(self, ip):
        # Find camera devices
        cameras = self.find_camera_devices()
        # Launch nodes with the discovered devices
        i = 1
        for camera in cameras:
            if i > 4:
                self.get_logger().info("Camera limit reached, not launching more nodes.")
                break
            else:
                self.get_logger().info(f"Launching node with camera: {camera}, to camera number: {i}")
                # Construct the command with f-strings
                cmd = [
                    "ros2",
                    "run", 
                    "videos", 
                    "videos_launch.py", 
                    "--ros-args", 
                    "-p", f"ip:={ip}", 
                    "-p", f"device:={camera}", 
                    "-p", f"camera_number:={i}", 
                    "-r", f"__node:=camera{i}"
                    ]
                # Run in a thread
                thread = threading.Thread(target=subprocess.run, args=(cmd,), kwargs={"check": True})
                thread.start()
                i += 1

    def find_camera_devices(self):
        # Run command: v4l2-ctl --list-devices
        output = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True).stdout
        lines = output.splitlines()
        cameras = []
        i = 0
        # Find the lines with "exploreHD" and get the third device
        while i < len(lines):
            if "exploreHD" in lines[i]:
                devices = []
                i += 1
                while i < len(lines) and lines[i].startswith("\t"):
                    devices.append(lines[i].strip())
                    i += 1
                if len(devices) >= 3:
                    cameras.append(devices[2])  # Third device (0-based index)
            else:
                i += 1

        self.get_logger().info(f"Discovered cameras: {cameras}")

        
    def publish_stop(self):
        msg = "STOP"

        self.publisher.publish(msg)
        self.stop_count += 1
        if self.stop_count >= self.stop_max_count:
            self.get_logger().info(f"Stopping publishing STOP")
            self.timer.cancel()


def main():
    rclpy.init()
    node = IpSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()