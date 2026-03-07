#! /usr/bin/python3
# import smbus
import math
import time

import rclpy
from rclpy.node import Node

from BNO085 import BNO085
from shared_msgs.msg import ImuMsg
from shared_msgs.msg import SensorCoordination
from MPU6050 import MPU6050


I2C_BUS = 1
IMU_ADDR = 0x68
REPORT_RATE = 1000000

class ImuSensor(Node):

    def __init__(self):
        super().__init__("imu_sensor")
        #self.publisher_ = self.create_publisher(Float64, "depth", 10)
        self.coord_publisher_ = self.create_publisher(SensorCoordination, "sensor_coordination", 10)
        self.coord_subscriber = self.create_subscription(SensorCoordination, "sensor_coordination", self.coord_callback, 10)
        self.i2c_status = []

        timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        try:
            self.sensor = MPU6050(1)
            self.sensor.init()  # Initializes with density of freshwater
            self.get_logger().info("IMU sensor connected")
        except:
            self.get_logger().info("IMU sensor not found")
            self.sensor = None
            exit(1)

    def timer_callback(self):
        if self.sensor.read():
            msg = ImuMsg()
            msg.gyro[0] = self.sensor.anglular_velocity_x
            msg.gyro[1] = self.sensor.anglular_velocity_y
            msg.gyro[2] = self.sensor.anglular_velocity_z
            msg.accel[0] = self.sensor.angluar_acceleration_x
            msg.accel[1] = self.sensor.linear_acceleration_y
            msg.accel[2] = self.sensor.linear_acceleration_z

            msg.pitch = self.sensor.pitch
            msg.roll = self.sensor.roll
            msg.yaw = self.sensor.yaw 
            self.get_logger().info(f"Pitch: {round(msg.pitch, 3)} Roll: {round(msg.roll, 3)} Yaw: {round(msg.yaw, 3)}")
            # publish imu data
            # self.publisher_.publish(msg)


           


def main(args=None):
    rclpy.init(args=args)
    imu_sensor = ImuSensor()

    # reset_imu_offsets()
    rclpy.spin(imu_sensor)

    imu_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
