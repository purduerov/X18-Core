#! /usr/bin/python3
import rclpy
from rclpy.node import Node

from shared_msgs.msg import FinalThrustMsg, ThrustStatusMsg, ThrustCommandMsg, ComMsg
from thrust_mapping import ThrustMapper

MAX_CHANGE = .15


class ThrustControlNode(Node):
    desired_p = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # desired thrust from pilot
    desired_p_unramped = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # pilot thrust converted to PWM
    desired_thrusters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # ramped pilot PWM; TODO: naming

    # TODO: not used?
    # desired_thrust_final = [0, 0, 0, 0, 0, 0]

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
        for i in range(0, 6):
            # self.desired_thrust_final[i] = self.desired_p[i]
            pass

        # calculate thrust
        # pwm_values = c.calculate(desired_thrust_final, disabled_list, False)
        self.desired_p_unramped = [self.tm.thrust_to_pwm(val) for val in self.tm.thruster_output(self.desired_p)]

        # invert relevant values
        # for i in range(8):
        #    if inverted_list[i] == 1:
        #        pwm_values[i] = pwm_values[i] * (-1)
        for i in range(0, 8):
            self.ramp(i)
        pwm_values = self.desired_thrusters  # TODO: temp variable assignment needed?

        # assign values to publisher messages for thrust control and status
        # val = float of range(-1, 1)
        # if int8: (val * 127.5) - 0.5 will give range -128 to 127
        # if uint8: (val + 1) * 127.5 will give 0 to 255
        thrusters = [127, 127, 127, 127, 127, 127, 127, 127]
        for i in range(0, 8):
            thrusters[i] = int((pwm_values[i] + 1) * 127.5)
            if thrusters[i] > 255:
                thrusters[i] = 255
            print(pwm_values)
            print(thrusters)

        tcm = FinalThrustMsg()
        tcm.thrusters = bytearray(thrusters)

        tsm = ThrustStatusMsg()
        tsm.status = pwm_values

        # publish data
        self.thrust_pub.publish(tcm)
        self.status_pub.publish(tsm)

    def _com_update(self, msg):
        self.tm.location = self.tm.change_origin(msg.com[0], msg.com[1], msg.com[2])
        self.tm.torque = self.tm.torque_values()
        self.tm.thruster_force_map = self.tm.thruster_force_map_values()
        self.get_logger().info("changed" + str(msg.com[0]) + ":" + str(msg.com[1]) + ":" + str(msg.com[2]))

    def ramp(self, index):
        if abs(self.desired_p_unramped[index] - self.desired_thrusters[index]) > MAX_CHANGE:
            if self.desired_p_unramped[index] - self.desired_thrusters[index] > 0:
                self.desired_thrusters[index] += MAX_CHANGE
                # print(index, "ramping", desired_thrusters[index])
            else:
                self.desired_thrusters[index] -= MAX_CHANGE
            return
        else:
            self.desired_thrusters[index] = self.desired_p_unramped[index]


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
