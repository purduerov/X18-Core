#!/usr/bin/env python3

# Import necessary libraries
import rclpy
import ipaddress
from rclpy.node import Node
from std_msgs.msg import String


class IpSubscriberNode(Node):
    def __init__(self):
        super().__init__('ip_subscriber_node')
        
        self.create_subscription(String, 'surface_ip', self.get_ip, 10)
        

    def get_ip(self, msg):
        received_ip = msg.data

        try:
            ipaddress.ipaddress(received_ip) 
            
            self.create_publisher(String, 'surface_ip', 10)
            self.timer = self.create_timer(1.0, self.publish_stop)
            self.stop_count = 0
            self.stop_max_count = 5
            self.get_logger().info(f'Received from surface_ip topic: "{msg.data}"')

            ## Subprocess command

        except ValueError:
            self.get_logger().info(f'Invalid IP received from topic: "{msg.data}"')

    def launch_camera(self, ip):
        pass

    def publish_stop(self):
        msg = "STOP"

        self.publisher.publish(msg)
        self.stop_count += 1
        if self.stop_count >= self.stop_max_count:
            self.get_logger().info(f"Stopping publishing STOP")
            self.timer.cancel()


def main():
    rclpy.init()
    IpSubscriberNode = IpSubscriberNode()
    rclpy.spin(IpSubscriberNode)
    IpSubscriberNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()