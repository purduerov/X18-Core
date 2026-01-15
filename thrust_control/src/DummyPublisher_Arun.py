<<<<<<< HEAD
=======
#! /usr/bin/python3
>>>>>>> cfbe5a8 (Innovative CRC working)
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

<<<<<<< HEAD
    delay = 0.1   # seconds (10 Hz) — change this to whatever you want
=======
    delay = 1 # seconds (10 Hz) — change this to whatever you want
>>>>>>> cfbe5a8 (Innovative CRC working)

    try:
        while rclpy.ok():
            msg = FinalThrustMsg()
<<<<<<< HEAD
            msg.thrust = [100, 100, 100, 100, 100, 100]  # example payload

            node.publisher.publish(msg)
            node.get_logger().info(f"Published thrust: {msg.thrust}")
=======
            msg.thrusters = [100, 100, 100, 100, 100, 100]  # example payload

            node.publisher.publish(msg)
            node.get_logger().info(f"Published thrust: {msg.thrusters}")
>>>>>>> cfbe5a8 (Innovative CRC working)

            time.sleep(delay)  # <-- THIS controls the loop rate

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
