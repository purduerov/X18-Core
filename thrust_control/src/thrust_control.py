#! /usr/bin/python3
import rclpy
from shared_msgs.msg import FinalThrustMsg, ThrustStatusMsg, ThrustCommandMsg, ComMsg
from thrust_mapping import ThrustMapper

desired_p = [0.0,0.0,0.0,0.0,0.0,0.0]
desired_thrusters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0]
desired_p_unramped = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
locked_dims_list = [False, False, False, False, False, False]
disabled_list = [False, False, False, False, False, False, False, False]
inverted_list = [0, 0, 0, 0, 0, 0, 0, 0]
desired_thrust_final = [0, 0, 0, 0, 0, 0]
MAX_CHANGE = .15
mode_fine = True
fine_multiplier = 1.041
tm = ThrustMapper()

global node
global thrust_pub
global status_pub

def _pilot_command(comm):
    global desired_p  # desired thrust from pilot
    global disabled_list  # disabled thrusters
    global inverted_list  # inverted thrusters
    global desired_p_unramped
    global tm
    #print (comm.desired_thrust)
    desired_p = comm.desired_thrust
    tm.set_multiplier(comm.multiplier)
    tm.set_fine(comm.is_fine)
    # disabled_list = comm.disable_thrusters
    # inverted_list = comm.inverted
    on_loop()

def ramp(index):
    if (abs(desired_p_unramped[index] - desired_thrusters[index]) > MAX_CHANGE):
        if (desired_p_unramped[index] - desired_thrusters[index] > 0):
            desired_thrusters[index] += MAX_CHANGE
            #print(index, "ramping", desired_thrusters[index])
        else:
            desired_thrusters[index] -= MAX_CHANGE
        return
    else:
        desired_thrusters[index] = desired_p_unramped[index]

def on_loop():
    global desired_thrusters
    global desired_p_unramped
    for i in range(0, 6):
        desired_thrust_final[i] = desired_p[i]

    # calculate thrust
    #pwm_values = c.calculate(desired_thrust_final, disabled_list, False)
    desired_p_unramped = [tm.thrust_to_pwm(val) for val in tm.thruster_output(desired_p)]
    # invert relevant values
    #for i in range(8):
    #    if inverted_list[i] == 1:
    #        pwm_values[i] = pwm_values[i] * (-1)
    for i in range(0,8):
        ramp(i)
    pwm_values = desired_thrusters
    
    # assign values to publisher messages for thurst control and status
    tcm = FinalThrustMsg()
    # val = float of range(-1, 1)
    # if int8: (val * 127.5) - 0.5 will give range -128 to 127
    # if uint8: (val + 1) * 127.5 will give 0 to 255
    thrusters = [127,127,127,127,127,127,127,127]
    for i in range(0,8):
        thrusters[i] = int((pwm_values[i] + 1) * 127.5)
        if thrusters[i] > 255:
            thrusters[i] = 255
        print(pwm_values)
        print(thrusters)

    tcm.thrusters = bytearray(thrusters)

    tsm = ThrustStatusMsg()
    tsm.status = pwm_values
    
    # publish data
    thrust_pub.publish(tcm)
    status_pub.publish(tsm)

def _comUpdate(msg):
    tm.location = tm.change_origin(msg.com[0],msg.com[1],msg.com[2])
    tm.torque = tm.torque_values()
    tm.thruster_force_map = tm.thruster_force_map_values()
    node.get_logger().info("changed" + str(msg.com[0]) + ":" + str(msg.com[1]) + ":" + str(msg.com[2]))

if __name__ == "__main__":
    '''
    Note that this file is only set up for using 8 thrusters.
    '''

    # initialize node and rate
    rclpy.init()

    node = rclpy.create_node('thrust_control')

    #srv = Server(ROV_COMConfig, updateCOM)
    rate = node.create_rate(25)  # 20 hz

    # initialize publishers
    thrust_pub = node.create_publisher(FinalThrustMsg, 'final_thrust', 10)
    status_pub = node.create_publisher(ThrustStatusMsg, 'thrust_status', 10)

    # initialize subscribers
    comm_sub = node.create_subscription(ThrustCommandMsg, '/thrust_command', _pilot_command, 10)
    com_sub = node.create_subscription(ComMsg, 'com_tweak', _comUpdate, 10)

    # create node parameters
    node.declare_parameter('ROV_X_scale', 10.0)
    node.declare_parameter('ROV_X', 0.1)
    node.declare_parameter('ROV_Y_scale', 10.0)
    node.declare_parameter('ROV_Y', 0.1)
    node.declare_parameter('ROV_Z_scale', 10.0)
    node.declare_parameter('ROV_Z', 0.1)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

