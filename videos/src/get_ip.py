#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from utils.heartbeat_helper import HeartbeatHelper
import subprocess, threading, ipaddress


class IpSubscriberNode(Node):
    def __init__(self):
        super().__init__('ip_subscriber_node')
        self.stop_max_count = 5 # Maximum number of times to publish STOP to the surface_ip topic
        self.stop_count = 0
        self.camera_limit = 4 # Maximum number of cameras to launch
        self.camera_device_num = 2 # The device number of the camera to launch

        # Set up heartbeat
        self.heartbeat_helper = HeartbeatHelper(self)
        
        # Create a subscriber to the surface_ip topic
        self.create_subscription(String, 'surface_ip', self.surface_ip_callback, 10)
        

    def surface_ip_callback(self, msg):
        received_ip = msg.data
        try:
            # Check if the received IP is valid
            ipaddress.ip_address(received_ip) 
            
            # Publish "STOP" to the surface_ip topic
            self.publisher = self.create_publisher(String, 'surface_ip', 10)
            self.timer = self.create_timer(1.0, self.publish_stop)
            self.get_logger().info(f'Received from surface_ip topic: "{msg.data}"')

            # Find the camera devices
            cameras = self.find_camera_devices()

            # Launch the camera nodes
            self.launch_camera(received_ip, cameras)

        except ValueError:
            self.get_logger().info(f'Invalid IP received from topic: "{msg.data}"')

    def launch_camera(self, ip, cameras):
        # Launch nodes with the discovered devices
        for camera, i in enumerate(cameras):
            if i > 4:
                self.get_logger().info("Camera limit reached, not launching more nodes.")
                break
            else:
                self.get_logger().info(f"Launching node with camera: {camera}, to camera number: {i + 1}")
                # Construct the command with f-strings
                cmd = [
                    "ros2", "run", "videos", "videos_launch.py", 
                    "--ros-args", 
                    "-p", f"ip:={ip}", 
                    "-p", f"device:={camera}", 
                    "-p", f"camera_number:={i}", 
                    "-r", f"__node:=camera{i}"
                    ]
                # Run in a thread
                thread = threading.Thread(target=subprocess.run, args=(cmd,), kwargs={"check": True}).start()


    def find_camera_devices(self):
        # Run command: v4l2-ctl --list-devices
        output = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True).stdout
        lines = output.splitlines()
        cameras = []
        i = 0
        # Find the lines with "exploreHD" and get the desired camera device
        while i < len(lines):
            if "exploreHD" in lines[i]:
                devices = []
                i += 1
                while i < len(lines) and lines[i].startswith("\t"):
                    devices.append(lines[i].strip())
                    i += 1
                if len(devices) >= self.camera_device_num + 1:
                    cameras.append(devices[self.camera_device_num])
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