#! /usr/bin/python3
import time

import rclpy
from rclpy.node import Node
from shared_msgs.msg import ImuMsg, ImuVelocityCommand


class ImuAngleControlNode(Node):
    kp = [0.0, 0.0, 0.0]  # TODO
    ki = [0.0, 0.0, 0.0]
    kd = [0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__("imu_angle_control")

        self.sub = self.create_subscription(ImuMsg, "imu", self.imu_reading, 1)
        self.pub = self.create_publisher(ImuVelocityCommand, "imu_vel_command", 10)

        self.prev_error = [0.0, 0.0, 0.0]  # Tuple of [pitch, roll, yaw]
        self.prev_ts = time.time()

        self.target_angle = [0.0, 0.0, 0.0]  # TODO: receiving this
        self.istate = [0.0, 0.0, 0.0]

    def imu_reading(self, msg):
        ts = time.time()

        out_msg = ImuVelocityCommand()
        output = [0.0, 0.0, 0.0]

        # PID on each of [pitch, roll, yaw], writing outputs to `self.output`.
        # Algorithm adopted from https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
        # (but without `kF` and `izone`).
        for i, angle in enumerate(msg.gyro):
            error = self.target_angle[i] - angle
            self.istate[i] += error * self.ki[i]
            deriv = (error - self.prev_error[i]) / (ts - self.prev_ts)

            p_out = self.kp[i] * error
            i_out = self.istate[i]
            d_out = self.kd[i] * deriv

            self.prev_error[i] = error
            output[i] = p_out + i_out + d_out

        out_msg.angular = output
        self.pub.publish(out_msg)

        self.prev_ts = ts


def main(args=None):
    rclpy.init(args=args)
    angle_pid = ImuAngleControlNode()

    rclpy.spin(angle_pid)

    angle_pid.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
