#! /usr/bin/python3
import rclpy
from rclpy.node import Node

from shared_msgs.msg import FinalThrustMsg, ThrustStatusMsg, ThrustCommandMsg, ComMsg, MotorMsg
from thrust_mapping import ThrustMapper
import numpy as np
from enum import Enum

MAX_CHANGE = .15

class multiplier(Enum):
    standard = 0
    fine = 1
    yeet = 2

class multiplierArrays(Enum):
    fine_multiplier = [1.5, 1.5, 1.5, 0.2, 1.0, 1.0]
    std_multiplier = [3, 3, 3, 0.4, 2.0, 2.0]
    yeet_multiplier = [15, 10, 10, 0.5, 3.5, 4.5]

class ThrustControlNode(Node):
    def __init__(self):
        super().__init__('thrust_control')

        self.rate = self.create_rate(25)
        self.tm = ThrustMapper()

        # initialize publishers
        self.thrust_pub = self.create_publisher(FinalThrustMsg, 'final_thrust', 10)
        self.status_pub = self.create_publisher(ThrustStatusMsg, 'thrust_status', 10)

        self.tools_pub = self.create_publisher(MotorMsg, 'motor_control', 10) # TODO: ADD THIS PART

        # initialize subscribers
        self.comm_sub = self.create_subscription(ThrustCommandMsg, '/thrust_command', self._pilot_command, 10)
        self.com_sub = self.create_subscription(ComMsg, 'com_tweak', self._com_update, 10)

        # create node parameters
        self.declare_parameter('ROV_X_scale', 10.0)
        self.declare_parameter('ROV_X', 0.1)
        self.declare_parameter('ROV_Y_scale', 10.0)
        self.declare_parameter('ROV_Y', 0.1)
        self.declare_parameter('ROV_Z_scale', 10.0)
        self.declare_parameter('ROV_Z', 0.1)

        #initialize thrust arrays
        self.desired_effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.desired_thrusters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.desired_thrusters_unramped = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.locked_dims_list = [False, False, False, False, False, False]
        self.disabled_list = [False, False, False, False, False, False, False, False]
        self.inverted_list = [0, 0, 0, 0, 0, 0, 0, 0]
        self.desired_thrust_final = [0, 0, 0, 0, 0, 0]

        self.power_mode = multiplier.standard

        self.fine_multiplier = [1.5, 1.5, 1.5, 0.2, 1.0, 1.0]
        self.std_multiplier = [3, 3, 3, 0.4, 2.0, 2.0]
        self.yeet_multiplier = [15, 10, 10, 0.5, 3.5, 4.5]

    def _pilot_command(self, comm):
        self.desired_p = comm.desired_thrust

        self.desired_effort = comm.desired_thrust
        self.tm.set_multiplier(comm.multiplier)
        
        self.power_mode =  comm.is_fine

        self.on_loop()

    def on_loop(self):
        # calculate thrust
        for i in range(0, 6):
            self.desired_thrust_final[i] = self.desired_effort[i]
        
        #desired_effort is 6 value vector of trans xyz, rot xyz
        if np.linalg.norm(self.desired_effort) > 1:
            self.desired_effort /= np.linalg.norm(self.desired_effort)
            # desired_effort is now normalized
        if self.power_mode == multiplier.fine:
            self.desired_effort = self.desired_effort * self.fine_multiplier
        elif self.power_mode == multiplier.yeet:
            self.desired_effort = self.desired_effort * self.yeet_multiplier
        else:
            self.desired_effort = self.desired_effort * self.std_multiplier

        # calculate thrust
        self.desired_thrusters_unramped = [self.tm.thrust_to_pwm(val) for val in self.tm.thruster_output(self.desired_effort)]

        self.ramp(self.desired_thrusters_unramped)
        pwm_values = self.desired_thrusters

        thrusters = [127, 127, 127, 127, 127, 127, 127, 127]
        for i in range(0, 8):
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

        tlm = MotorMsg() # TODO: ADDED THIS PART
        tools = [127, 127, 127, 127]
        tlm.tools = tools

        # publish data
        self.thrust_pub.publish(tcm)
        self.status_pub.publish(tsm)

        self.tools_pub.publish(tlm) # TODO: ADDED THIS PART

    def _com_update(self, msg):
        self.tm.location = self.tm.change_origin(msg.com[0], msg.com[1], msg.com[2])
        self.tm.torque = self.tm.torque_values()
        self.tm.thruster_force_map = self.tm.thruster_force_map_values()
        self.get_logger().info("changed" + str(msg.com[0]) + ":" + str(msg.com[1]) + ":" + str(msg.com[2]))

    def ramp(self, unramped_thrusters):
        for index in range(0,8):
            if abs(unramped_thrusters[index] - self.desired_thrusters[index]) > MAX_CHANGE:
                if unramped_thrusters[index] - self.desired_thrusters[index] > 0:
                    self.desired_thrusters[index] += MAX_CHANGE
                else:
                    self.desired_thrusters[index] -= MAX_CHANGE
                return
            else:
                self.desired_thrusters[index] = unramped_thrusters[index]


def main(args=None):
    rclpy.init(args=args)
    thrust_control = ThrustControlNode()

    rclpy.spin(thrust_control)

    thrust_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    '''
    Note that this file is only set up for using 8 thrusters.
    '''
    main()
