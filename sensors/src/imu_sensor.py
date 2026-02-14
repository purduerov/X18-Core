#! /usr/bin/python3
# import smbus
import math
import time

import rclpy
from rclpy.node import Node

from BNO085 import BNO085
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

I2C_BUS = 1
IMU_ADDR = 0x00
REPORT_RATE = 1000000

class ImuSensor(Node):
    IMU_PITCH_OFFSET = 0.0
    IMU_ROLL_OFFSET = 0.0
    IMU_YAW_OFFSET = 0.0

    def __init__(self):
        super().__init__("imu_sensor")

        self.imu = BNO085(I2C_BUS, IMU_ADDR, 0)
        self.imu.setup(REPORT_RATE)
        time.sleep(1)
        self.pub = self.create_publisher(ImuMsg, "imu", 1)

        self.timer = self.create_timer(1 / 20.0, self.loop)

    def loop(self):
        if self.imu.read():
            out_message = ImuMsg()

            # convert everything to a 0 to 360 to apply a 1d rotation then convert back to -180 to 180
            rov_pitch = clamp_angle_0_to_360(self.imu.rotation_data[0]) - self.IMU_ROLL_OFFSET
            rov_roll = clamp_angle_0_to_360(self.imu.rotation_data[1]) - self.IMU_YAW_OFFSET
            rov_yaw = clamp_angle_0_to_360(self.imu.rotation_data[2]) - self.IMU_PITCH_OFFSET

            out_message.gyro = [rov_pitch, rov_roll, rov_yaw]

            rov_x_accel = self.imu.accel_data[0]
            rov_y_accel = self.imu.accel_data[1]
            rov_z_accel = self.imu.accel_data[2]

            self.get_logger().info(f"Accel: {rov_x_accel, rov_y_accel, rov_z_accel}\n Rotation {rov_pitch, rov_yaw, rov_roll}")
            out_message.accel = [rov_x_accel, rov_y_accel, rov_z_accel]

            self.pub.publish(out_message)


def main(args=None):
    rclpy.init(args=args)
    imu_sensor = ImuSensor()

    # reset_imu_offsets()
    rclpy.spin(imu_sensor)

    imu_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
