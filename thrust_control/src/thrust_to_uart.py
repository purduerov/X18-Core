import rclpy
import time
from rclpy.node import Node
from crccheck.crc import Crc32Mpeg2

from shared_msgs.msg import FinalThrustMsg, ToolsMotorMsg

class ThrustToUARTNode(node):
    def __init__(self):
        super().__init__("thrust_to_uart")

        logger = self.get_logger().info("UART INITIALIZED")

        return

def main(args=None):
    rclpy.init(args=args)
    node = ThrustToUARTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handler()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()