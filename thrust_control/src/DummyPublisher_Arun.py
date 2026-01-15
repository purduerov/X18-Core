#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from shared_msgs.msg import FinalThrustMsg
import time


class FinalThrustPublisher(Node):
    def __init__(self):
        super().__init__('final_thrust_publisher')

        self.publisher = self.create_publisher(
            FinalThrustMsg,
            'final_thrust',
            10
        )


def main(args=None):
    rclpy.init(args=args)
    node = FinalThrustPublisher()

    delay = 1 # seconds (10 Hz) — change this to whatever you want

    try:
        while rclpy.ok():
            msg = FinalThrustMsg()
            msg.thrusters = [100, 100, 100, 100, 100, 100]  # example payload

            node.publisher.publish(msg)
            node.get_logger().info(f"Published thrust: {msg.thrusters}")

            time.sleep(delay)  # <-- THIS controls the loop rate

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
