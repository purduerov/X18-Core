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

MAX_CHANGE = 2

# x trans ability is capped at 18.33 kgf-m physically
# y trans ability is capped at 10.59 kgf-m physically
# z trans ability is capped at 9.87 kgf-m physically
# x rot ability is capped at 0.59 kgf-m physically
# y rot ability is capped at 4.0 kgf-m physically
# z rot ability is capped at 5.39 kgf-m physically


class multiplier(Enum):
    fine = 0
    standard = 1
    yeet = 2
    MEGAYEET = 3


class reference_frame(Enum):
    body = 0
    spatial = 1


# [x trans, y trans, z trans, x rot, y rot, z rot]
MULT_DICT = {
    multiplier.fine : [1.0, 1.0, 1.0, 0.2, 0.6, 0.4], multiplier.standard : [1.5, 1.5, 1.5, 0.2, 1.0, 1.0],
    multiplier.yeet : [3, 3, 3, 0.4, 2.0, 2.0], multiplier.MEGAYEET : [6, 6, 6, 0.4, 2.0, 2.0]
    }


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
        self.desired_thrusters = np.asarray([0, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.desired_thrusters_unramped = np.asarray([0, 0, 0, 0, 0, 0], dtype=np.uint8)

        self.power_mode = multiplier.standard
        self.frame = reference_frame.body

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
        global MULT_DICT

        # desired_effort is 6 value vector of trans xyz, rot xyz
        if np.linalg.norm(self.desired_effort) > 1:
            self.desired_effort /= np.linalg.norm(self.desired_effort)
        # self.get_logger().info("pre-ramped desired_effort: " + str(self.desired_effort))


        # scale effort by multplier value
        self.desired_effort *= MULT_DICT[self.power_mode]

        # self.get_logger().info("desired_effort: " + str(self.desired_effort))

        # calculate thrust
        self.desired_thrusters_unramped = self.tm.get_pwm(self.desired_effort)

        self.ramp(self.desired_thrusters_unramped)
        pwm_values = self.desired_thrusters
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

def main(args=None):
    rclpy.init(args=args)
    thrust_control = ThrustControlNode()

    rclpy.spin(thrust_control)

    thrust_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    """
    Note that this file is only set up for using 8 thrusters.
    """
    main()
