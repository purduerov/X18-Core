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

TEMP_COMPLETE = (False, False, True, True)
IMU_READING = (True, False, False, False)
IMU_COMPLETE = (True, False, False, True)

class ImuSensor(Node):

    def __init__(self):
        super().__init__("imu_sensor")
        self.publisher_ = self.create_publisher(ImuMsg, "imu_msg", 10)
        self.coord_publisher_ = self.create_publisher(SensorCoordination, "sensor_coordination", 10)
        self.coord_subscriber = self.create_subscription(SensorCoordination, "sensor_coordination", self.coord_callback, 10)
        self.i2c_status = []

        timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        try:
            self.sensor = MPU6050(I2C_BUS, IMU_ADDR, 0)
            self.sensor.setup()  # Initializes with density of freshwater
            self.get_logger().info("IMU sensor connected")
        except:
            self.get_logger().info("IMU sensor not found")
            self.sensor = None
            exit(1)

    def timer_callback(self):
        msg = ImuMsg()

        if(self.i2c_status == TEMP_COMPLETE):
            self.i2c_status = IMU_READING
            self.coord_publisher_.publish(self.i2c_status)
            
            self.sensor.read_imu()
            msg.gyro = self.sensor.gyro_data
            msg.accel = self.sensor.accel_data

            self.publisher_.publish(msg)
        elif(self.i2c_status == IMU_READING):
            self.i2c_status = IMU_COMPLETE
            self.coord_publisher_.publish(self.i2c_status)


        
    def coord_callback(self, msg):
        self.i2c_status = tuple(msg.i2c_status)

def main(args=None):
    rclpy.init(args=args)
    imu_sensor = ImuSensor()

    # reset_imu_offsets()
    rclpy.spin(imu_sensor)

    imu_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
