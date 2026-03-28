#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from shared_msgs.msg import ToolsCommandMsg

class ToolsDebugNode(Node):
    def __init__(self):
        super().__init__('tools_debug')
        self.get_logger().info("Tools Debug Node Started")
        # Subscribe to the tools_control topic
        self.sub = self.create_subscription(
            ToolsCommandMsg,
            'tools_control',
            self.tools_callback,
            10
        )

    def tools_callback(self, msg: ToolsCommandMsg):
        vertical, horizontal, claw, _ = msg.tools
        self.get_logger().info(
            f"[CORE RECEIVED] Vertical: {vertical} | Horizontal: {horizontal} | Claw: {claw}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ToolsDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()