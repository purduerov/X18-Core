#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from utils.heartbeat_helper import HeartbeatHelper


from shared_msgs.msg import (
    FinalThrustMsg,
    ThrustStatusMsg,
    ThrustCommandMsg,
    ComMsg,
    ImuMsg,
)  # , ToolsCommandMsg
from thrust_mapping import ThrustMapper
import numpy as np
from enum import Enum

MAX_CHANGE = 0.30

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
        self.status_pub = self.create_publisher(ThrustStatusMsg, "thrust_status", 10)

        # self.tools_pub = self.create_publisher(ToolsCommandMsg, 'tools_control', 10) # TODO: ADD THIS PART

        # initialize subscribers
        self.command_sub = self.create_subscription(
            ThrustCommandMsg, "thrust_command", self._pilot_command, 10
        )

        # initialize thrust arrays
        self.desired_effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.desired_thrusters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.desired_thrusters_unramped = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # set initial orientation matrix to 3x3
        self.orientation_matrix = np.identity(3)

        self.power_mode = multiplier.standard
        self.frame = reference_frame.body

    def _pilot_command(self, data):
        self.desired_effort = data.desired_thrust
        self.power_mode = data.is_fine
        # self.get_logger().info("power_mode: " + str(self.power_mode))

        if data.is_pool_centric:
            # self.frame = reference_frame.spatial
            pass
        else:
            self.frame = reference_frame.body

        self.on_loop()


    def on_loop(self):
        global MULT_DICT

        if self.frame == reference_frame.spatial:
            translational_effort = np.array(self.desired_effort[0:3])
            translational_effort = self.orientation_matrix.dot(translational_effort)

            self.desired_effort[0:3] = translational_effort
        #  self.get_logger().info("desired_effort effort: " + str(self.desired_effort))

        # desired_effort is 6 value vector of trans xyz, rot xyz
        if np.linalg.norm(self.desired_effort) > 1:
            self.desired_effort /= np.linalg.norm(self.desired_effort)
        # self.get_logger().info("pre-ramped desired_effort: " + str(self.desired_effort))


        # scale effort by multplier value
        self.desired_effort = MULT_DICT[self.power_mode]

        # self.get_logger().info("desired_effort: " + str(self.desired_effort))

        # calculate thrust
        self.desired_thrusters_unramped = [
            self.tm.thrust_to_pwm(val)
            for val in self.tm.thruster_output(self.desired_effort)
        ]

        self.ramp(self.desired_thrusters_unramped)
        pwm_values = self.desired_thrusters
        # self.get_logger().info("pwm: " + str(pwm_values))

        thrusters = [127, 127, 127, 127, 127, 127]
        for i in range(0, 6):
            thrusters[i] = int((pwm_values[i] + 1) * 127.5)
            if thrusters[i] > 255:
                thrusters[i] = 255
            elif thrusters[i] < 0:
                thrusters[i] = 0

        # assign values to publisher messages for thurst control and status
        tcm = FinalThrustMsg()

        tcm.thrusters = bytearray(thrusters)

        tsm = ThrustStatusMsg()
        tsm.status = pwm_values

        # tlm = ToolsCommandMsg() # TODO: ADDED THIS PART
        # tools = [127, 127, 127, 127]
        # tlm.tools = tools

        # publish data
        self.thrust_pub.publish(tcm)
        self.status_pub.publish(tsm)

        # self.tools_pub.publish(tlm)

    def ramp(self, unramped_thrusters):
        for index in range(0, 8):
            if (
                abs(unramped_thrusters[index] - self.desired_thrusters[index])
                > MAX_CHANGE
            ):
                if unramped_thrusters[index] - self.desired_thrusters[index] > 0:
                    self.desired_thrusters[index] += MAX_CHANGE
                else:
                    self.desired_thrusters[index] -= MAX_CHANGE
                # return
            else:
                self.desired_thrusters[index] = unramped_thrusters[index]


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
