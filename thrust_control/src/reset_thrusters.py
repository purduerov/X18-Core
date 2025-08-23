#! /usr/bin/python3

from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
import signal
import os, sys
from utils.heartbeat_helper import HeartbeatHelper
from time import sleep
from gpiozero import OutputDevice


class ResetThrustersNode(Node):
    def __init__(self):
        super().__init__("reset_thrusters_node")

        # Setup heartbeat
        self.heartbeat_helper = HeartbeatHelper(self)

        self.thruster_reset_pin = OutputDevice(5, active_high=True, initial_value=True)

        # Initalize subscriber
        self.create_subscription(Bool, "reset_thrusters", self.reset_thrusters, 10)

        self.get_logger().info("Reset node started")

    def reset_thrusters(self, msg):
        self.get_logger().info(f"Received thruster reset command: {msg.data}")
        self.thruster_reset_pin.off()
        sleep(1)
        self.thruster_reset_pin.on()


def main():
    rclpy.init()
    reset_thrusters_node = ResetThrustersNode()

    # Silent exit on SIGINT - just terminate immediately
    def silent_exit(sig, frame):
        os._exit(0)  # exit immediately without cleanup

    # Register the signal handler
    signal.signal(signal.SIGINT, silent_exit)
    signal.signal(signal.SIGTERM, silent_exit)

    try:
        rclpy.spin(reset_thrusters_node)
    except KeyboardInterrupt:
        # Exit silently on KeyboardInterrupt too
        os._exit(0)
    finally:
        # This will only run if spin() returns normally (which it shouldnt), not after os._exit()
        sys.exit(0)


if __name__ == "__main__":
    main()
