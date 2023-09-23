#! /usr/bin/python3
import rclpy
from rclpy.node import Node

from shared_msgs.msg import ImuMsg, ImuVelocityCommand


class ImuAngleControlNode(Node):
    angle = [0.0, 0.0, 0.0]  # Tuple of [pitch, roll, yaw]
    prev_angle = [0.0, 0.0, 0.0]
    target_angle = [0.0, 0.0, 0.0]  # TODO: receiving this

    kp = [0.0, 0.0, 0.0]  # TODO
    ki = [0.0, 0.0, 0.0]
    kd = [0.0, 0.0, 0.0]
    istate = [0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__('imu_angle_control')

        self.sub = self.create_subscription(ImuMsg, 'imu', self._com_update, 1)
        self.pub = self.create_publisher(ImuVelocityCommand, 'imu_vel_command', 10)

        self.period = 0.02
        self.timer = self.create_timer(self.period, self.periodic)

    def imu_reading(self, msg):
        self.angle = msg.gyro

    def periodic(self):
        msg = ImuVelocityCommand()
        output = [0.0, 0.0, 0.0]

        # PID on each of [pitch, roll, yaw], writing outputs to `self.output`
        for i in range(len(self.angle)):
            error = self.target_angle[i] - self.angle[i]
            self.istate[i] += error * self.ki[i]
            deriv = (self.angle[i] - self.prev_angle[i]) / self.period

            p_out = self.kp[i] * error
            i_out = self.istate[i]
            d_out = self.kd[i] * deriv

            output[i] = p_out + i_out + d_out

        msg.angular = output


def main(args=None):
    rclpy.init(args=args)
    angle_pid = ImuAngleControlNode()

    rclpy.spin(angle_pid)

    angle_pid.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
