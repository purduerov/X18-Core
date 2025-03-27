#! /usr/bin/python3

from rclpy.node import Node
from std_msgs.msg import Header

class HeartbeatHelper:
    def __init__(self, node: Node, topic_name="heartbeat", timer_period=5.0):
        """This class is used to publish a heartbeat message on a given topic at a given interval"""
        self.timer_period = timer_period
        self.node = node
        self.publisher = self.node.create_publisher(Header, topic_name, 10)
        self.timer = self.node.create_timer(timer_period, self.publish_heartbeat)

    def publish_heartbeat(self):
        """Publish a heartbeat message on the topic"""
        self.node.get_logger().info("Publishing heartbeat from {}".format(self.node.get_name()))
        msg = Header()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.frame_id = self.node.get_name()
        self.publisher.publish(msg)

    def stop_heartbeat(self):
        """Stop the heartbeat timer"""
        self.timer.cancel()
        self.node.destroy_publisher(self.publisher)
        self.node.get_logger().info(f'Heartbeat stopped for {self.node.get_name()}')

    def start_heartbeat(self):
        """Start the heartbeat timer"""
        self.timer = self.node.create_timer(self.timer_period, self.publish_heartbeat)
        self.node.get_logger().info(f'Heartbeat started for {self.node.get_name()}')
