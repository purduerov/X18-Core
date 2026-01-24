#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from shared_msgs.msg import FinalThrustMsg,ToolsMotorMsg
import time


class FinalThrustPublisher(Node):
    def __init__(self):
        super().__init__('final_thrust_publisher')

        self.thrust_ublisher = self.create_publisher(
            FinalThrustMsg,
            'final_thrust',
            10
        )

        self.tools_publisher = self.create_publisher(
            ToolsMotorMsg,
            'tools_motor',
            10
        )

        self.sub = self.create_subscription(
            FinalThrustMsg,
            'thrust_response',
            self.sub_callback,
            10
        )

        self.thrust = 6 * [0x64]
        self.tools = 6 * [0xAA]

        # self.timer = self.create_timer(1.0, self.publish_thrust)
        self.timer = self.create_timer(1.0, self.publish_tools)

    def sub_callback(self, msg):
        msg = list(msg.thrusters)
        self.get_logger().info(f"Response message from thrust_to_spi {msg}")
    
    def publish_thrust(self):
        msg = FinalThrustMsg()
        msg.thrusters = self.thrust
        self.thrust_publisher.publish(msg)
    
    def publish_tools(self):
        msg = ToolsMotorMsg()
        msg.tools = self.tools
        self.tools_publisher.publish(msg)

        
    


def main(args=None):
    rclpy.init(args=args)
    node = FinalThrustPublisher()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        


if __name__ == '__main__':
    main()
