#! /usr/bin/python3
import rclpy
from rclpy.node import Node

from shared_msgs.msg import ThrustCommandMsg, RovVelocityCommand, ImuVelocityCommand, ToolsCommandMsg, ToolsMotorMsg, FinalThrustMsg
from utils.heartbeat_helper import HeartbeatHelper


class ROVMainNode(Node):
    controller_percent_power = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    mode_fine = 0


    def __init__(self):
        super().__init__("ROV_main")

        # Heartbeat
        self.heartbeat_helper = HeartbeatHelper(self)

        self.controller_sub = self.create_subscription(
            RovVelocityCommand, "rov_velocity", self._controller_input, 10
        )

        self.thrust_command_pub = self.create_publisher(
            ThrustCommandMsg, "thrust_command", 10
        )

        self.thrust_sub = self.create_subscription(
            FinalThrustMsg,
            'thrust_response',
            self.thrust_response_callback,
            10
        )
        
        self.controller_percent_power[0] = 1
        self.controller_percent_power[1] = 1
        self.controller_percent_power[2] = 1
        self.controller_percent_power[3] = 1
        self.controller_percent_power[4] = 1
        self.controller_percent_power[5] = 1
        self.mode_fine = 1
        self.is_pool_centric = True

        self.timer = self.create_timer(1 / 15.0, self.on_loop)
        self.is_pool_centric = False

    def on_loop(self):
        # Thruster Control
        thrust_command = ThrustCommandMsg()
        thrust_command.desired_thrust = self.controller_percent_power

        # If set, override controller angular input with IMU PID loop values
        #      if self.imu_angle_lock_enable:
        #         thrust_command.desired_thrust[3:6] = self.imu_velocity

        thrust_command.is_fine = self.mode_fine
        thrust_command.is_pool_centric = self.is_pool_centric

        self.thrust_command_pub.publish(thrust_command)
        # self.tools_command_pub.publisher()

    def _controller_input(self, msg):
        self.controller_percent_power[0] = msg.twist.linear.x
        self.controller_percent_power[1] = msg.twist.linear.y
        self.controller_percent_power[2] = msg.twist.linear.z
        self.controller_percent_power[3] = msg.twist.angular.x
        self.controller_percent_power[4] = msg.twist.angular.y
        self.controller_percent_power[5] = msg.twist.angular.z
        self.mode_fine = msg.is_fine
        self.is_pool_centric = msg.is_pool_centric

    def _imu_input(self, msg):
        self.imu_velocity = msg.angular

    def thrust_response_callback(self, msg):
        # EDIT LATER
        # fill in once we know how surface will work
        return
        


def main(args=None):
    rclpy.init(args=args)
    node = ROVMainNode()
    print("We're so done")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
