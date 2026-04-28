#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from utils.heartbeat_helper import HeartbeatHelper


from shared_msgs.msg import (
    FinalThrustMsg,
    ThrustStatusMsg,
    ThrustCommandMsg,
    ToolsMotorMsg,
    ToolsCommandMsg
)
from thrust_mapping import ThrustMapper
import numpy as np
from enum import Enum

MAX_CHANGE = 5

# x trans ability is capped at 18.33 kgf-m physically
# y trans ability is capped at 10.59 kgf-m physically
# z trans ability is capped at 9.87 kgf-m physically
# x rot ability is capped at 0.59 kgf-m physically
# y rot ability is capped at 4.0 kgf-m physically
# z rot ability is capped at 5.39 kgf-m physically


FINE = 0
STANDARD = 1
YEET = 2
MEGAYEET = 3



# [x trans, y trans, z trans, x rot, y rot, z rot]
BASE = np.asarray([1.6, 1.6, 1.6, 10.0, 0.3, 0.6])
MULT = np.asarray([1.0 * BASE, 1.5 * BASE, 3.0 * BASE, 4.5 * BASE], dtype=float)


class ThrustControlNode(Node):
    def __init__(self):
        super().__init__("thrust_control")
        self.tm = ThrustMapper()

        # Setup heartbeat
        self.heartbeat_helper = HeartbeatHelper(self)

        # initialize publishers
        self.thrust_pub = self.create_publisher(FinalThrustMsg, "final_thrust", 10)

        self.tools_pub = self.create_publisher(ToolsMotorMsg, 'tools_motor', 10)

        # initialize subscribers
        self.command_sub = self.create_subscription(
            ThrustCommandMsg, "thrust_command", self._pilot_command, 10
        )

        self.tools_sub = self.create_subscription(
            ToolsCommandMsg, "tools_command", self._tools_command, 10
        )

        # initialize thrust arrays
        self.desired_effort = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        self.desired_thrusters = np.asarray([127, 127, 127, 127, 127, 127], dtype=np.int16)
        self.desired_thrusters_unramped = np.asarray([127, 127, 127, 127, 127, 127], dtype=np.int16)

        self.power_mode = MEGAYEET 

    def _pilot_command(self, data):
        self.desired_effort = data.desired_thrust
        self.power_mode = data.is_fine
        # self.get_logger().info("power_mode: " + str(self.power_mode))

        self.on_loop()

    def _tools_command(self, data):
        tools_command = np.asarray(data.tools)
        tools_command = np.clip(tools_command, 0, 255)
        msg = ToolsMotorMsg()
        msg.tools = list(tools_command)
        self.tools_pub.publish(msg)


    def on_loop(self):
        # scale effort by multplier value
        self.desired_effort *= (MULT[self.power_mode]  * 5)

        # self.get_logger().info("desired_effort: " + str(self.desired_effort))
        
        # self.desired_effort = np.asarray([20, 20, 20, 20, 20, 20], dtype=np.int16)
        # calculate thrust
        self.desired_thrusters_unramped = self.tm.get_pwm(self.desired_effort)

        self.ramp(self.desired_thrusters_unramped)
        pwm_values = self.desired_thrusters_unramped
        # self.get_logger().info("pwm: " + str(pwm_values))

        # assign values to publisher messages for thurst control and status
        tcm = FinalThrustMsg()

        tcm.thrusters = pwm_values


        # publish data
        self.thrust_pub.publish(tcm)


    # pwm cannot change by more than MAX_CHANGE per command
    def ramp(self, unramped_thrusters):
        # calculate the difference between the new and old thruster values
        diff = unramped_thrusters - self.desired_thrusters
        # clip the difference to +- MAX_CHANGE
        diff = np.clip(diff, -MAX_CHANGE, MAX_CHANGE)
        # add the difference back into the old thruster values
        self.desired_thrusters += diff
        self.desired_thrusters = np.clip(self.desired_thrusters, 0, 255)

def main(args=None):
    rclpy.init(args=args)
    thrust_control = ThrustControlNode()

    rclpy.spin(thrust_control)

    thrust_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    """
    Note that this file is only set up for using 6 thrusters.
    """
    main()
