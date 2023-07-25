#! /usr/bin/python3
import rclpy
from shared_msgs.msg import ThrustCommandMsg, RovVelocityCommand

controller_percent_power = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
controller_tools_command = [0,0,0,0]
translation_Scaling = 3.2
rotation_Scaling = 1.5
mode_fine = True
fine_multiplier = 1.041

global node
global thrust_command_pub

def onLoop():
    #Thruster Control
    thrust_command = ThrustCommandMsg()
    thrust_command.desired_thrust = controller_percent_power
    thrust_command.is_fine = mode_fine
    thrust_command.multiplier = fine_multiplier
    thrust_command_pub.publish(thrust_command)

def _velocity_input(msg):
    global mode_fine, fine_multiplier
    controller_percent_power[0] = msg.twist.linear.x
    controller_percent_power[1] = msg.twist.linear.y
    controller_percent_power[2] = msg.twist.linear.z
    controller_percent_power[3] = msg.twist.angular.x
    controller_percent_power[4] = msg.twist.angular.y
    controller_percent_power[5] = msg.twist.angular.z
    mode_fine = msg.is_fine
    fine_multiplier = msg.multiplier

if __name__ == '__main__':
    rclpy.init()

    node = rclpy.create_node('ROV_main')
    velocity_sub = node.create_subscription(RovVelocityCommand, '/rov_velocity', _velocity_input, 10)
    thrust_command_pub = node.create_publisher(ThrustCommandMsg, '/thrust_command', 10)
    timer = node.create_timer(1 / 50.0, onLoop)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
