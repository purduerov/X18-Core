#! /usr/bin/python3
# import smbus
import math
import time

import rclpy
from rclpy.node import Node

from BNO055 import BNO055
from shared_msgs.msg import ImuMsg


# bind all angles to -180 to 180
def clamp_angle_neg180_to_180(angle):
    angle_0_to_360 = clamp_angle_0_to_360(angle)
    if angle_0_to_360 > 180:
        return angle_0_to_360 - 180 * -1.0
    return angle_0_to_360


# bind all angles to 0 to 360
def clamp_angle_0_to_360(angle):
    return (angle + 1 * 360) - math.floor((angle + 2 * 360) / 360) * 360


class ImuSensor(Node):
    IMU_PITCH_OFFSET = 0.0
    IMU_ROLL_OFFSET = 0.0
    IMU_YAW_OFFSET = 0.0

    def __init__(self):
        super().__init__('imu_sensor')

        self.imu = BNO055()
        time.sleep(1)
        self.pub = self.create_publisher(ImuMsg, 'imu', 1)

        self.timer = self.create_timer(1 / 20.0, self.loop)

    def loop(self):
        if self.imu.update():
            out_message = ImuMsg()

            # convert everything to a 0 to 360 to apply a 1d rotation then convert back to -180 to 180
            rov_pitch = clamp_angle_0_to_360(self.imu.roll()) - self.IMU_ROLL_OFFSET
            rov_roll = clamp_angle_0_to_360(self.imu.pitch()) - self.IMU_YAW_OFFSET
            rov_yaw = clamp_angle_0_to_360(self.imu.yaw()) - self.IMU_PITCH_OFFSET

            out_message.gyro = [rov_pitch, rov_roll, rov_yaw]

            rov_x_accel = self.imu.acceleration_x()
            rov_y_accel = self.imu.acceleration_y()
            rov_z_accel = self.imu.acceleration_z()
            out_message.accel = [rov_x_accel, rov_y_accel, rov_z_accel]

            self.pub.publish(out_message)

    def reset_imu_offsets(self):
        self.imu.update()

        self.IMU_PITCH_OFFSET = self.imu.pitch()
        self.IMU_ROLL_OFFSET = self.imu.roll()
        self.IMU_YAW_OFFSET = self.imu.yaw()


def main(args=None):
    rclpy.init(args=args)
    imu_sensor = ImuSensor()

    # reset_imu_offsets()
    rclpy.spin(imu_sensor)

    imu_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
