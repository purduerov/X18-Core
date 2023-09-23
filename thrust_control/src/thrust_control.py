#! /usr/bin/python3
import rclpy
from rclpy.node import Node

from shared_msgs.msg import FinalThrustMsg, ThrustStatusMsg, ThrustCommandMsg, ComMsg
from thrust_mapping import ThrustMapper

MAX_CHANGE = .15


class ThrustControlNode(Node):
    # Pilot input, as a tuple of [X, Y, Z, Roll, Pitch, Yaw]
    desired_p = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Unramped PWM values for each thruster; TODO: eliminate?
    unramped_thruster_pwm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    final_thruster_pwm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # TODO: not used?
    # disabled_list = [False, False, False, False, False, False, False, False]  # disabled thrusters
    # inverted_list = [0, 0, 0, 0, 0, 0, 0, 0]  # inverted thrusters

    def __init__(self):
        super().__init__('thrust_control')

        # srv = Server(ROV_COMConfig, updateCOM)
        self.rate = self.create_rate(25)  # 20 hz
        self.tm = ThrustMapper()

        # initialize publishers
        self.thrust_pub = self.create_publisher(FinalThrustMsg, 'final_thrust', 10)
        self.status_pub = self.create_publisher(ThrustStatusMsg, 'thrust_status', 10)

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

    def _pilot_command(self, comm):
        self.desired_p = comm.desired_thrust
        self.tm.set_multiplier(comm.multiplier)
        self.tm.set_fine(comm.is_fine)
        # disabled_list = comm.disable_thrusters
        # inverted_list = comm.inverted

        self.on_loop()

    def on_loop(self):
        # calculate thrust
        self.unramped_thruster_pwm = [self.tm.thrust_to_pwm(val) for val in self.tm.thruster_output(self.desired_p)]

        # invert relevant values
        # for i in range(8):
        #    if inverted_list[i] == 1:
        #        pwm_values[i] = pwm_values[i] * (-1)
        for i in range(0, 8):
            self.ramp(i)

        # assign values to publisher messages for thrust control and status
        # val = float of range(-1, 1)
        # if int8: (val * 127.5) - 0.5 will give range -128 to 127
        # if uint8: (val + 1) * 127.5 will give 0 to 255
        thrusters = [127, 127, 127, 127, 127, 127, 127, 127]
        for i in range(0, 8):
            thrusters[i] = int((self.final_thruster_pwm[i] + 1) * 127.5)
            if thrusters[i] > 255:
                thrusters[i] = 255
            print(self.final_thruster_pwm)
            print(thrusters)

        tcm = FinalThrustMsg()
        tcm.thrusters = bytearray(thrusters)

        tsm = ThrustStatusMsg()
        tsm.status = self.final_thruster_pwm

        # publish data
        self.thrust_pub.publish(tcm)
        self.status_pub.publish(tsm)

    def _com_update(self, msg):
        self.tm.location = self.tm.change_origin(msg.com[0], msg.com[1], msg.com[2])
        self.tm.torque = self.tm.torque_values()
        self.tm.thruster_force_map = self.tm.thruster_force_map_values()
        self.get_logger().info("changed" + str(msg.com[0]) + ":" + str(msg.com[1]) + ":" + str(msg.com[2]))

    def ramp(self, index):
        if abs(self.unramped_thruster_pwm[index] - self.final_thruster_pwm[index]) > MAX_CHANGE:
            if self.unramped_thruster_pwm[index] - self.final_thruster_pwm[index] > 0:
                self.final_thruster_pwm[index] += MAX_CHANGE
                # print(index, "ramping", desired_thrusters[index])
            else:
                self.final_thruster_pwm[index] -= MAX_CHANGE
            return
        else:
            self.final_thruster_pwm[index] = self.unramped_thruster_pwm[index]


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
