#!/usr/bin/env python3

# Import necessary libraries
import rclpy
import ipaddress
from rclpy.node import Node
from std_msgs.msg import String


class IpSubscriberNode(Node):
    def __init__(self):
        super().__init__('ip_subscriber_node')
        ## TODO change the topic name to the related topic
        self.create_subscription(String, 'TODO', self.get_ip, 10)

    def get_ip(self, msg):
        received_ip = msg.data

        try:
            ipaddress.ipaddress(received_ip) 
            ## TODO change the topic name to the related topic
            self.get_logger().info(f'Received from TODO topic: "{msg.data}"')
        except ValueError:
            self.get_logger().info(f'Invalid IP received from topic: "{msg.data}"')

    def launch_camera(self, ip):
        pass


def main():
    rclpy.init()
    IpSubscriberNode = IpSubscriberNode()
    rclpy.spin(IpSubscriberNode)
    IpSubscriberNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()