#! /usr/bin/python3
import rclpy
# import smbus
import math
import time
import numpy as np
from BNO055 import BNO055
import board
import time
from shared_msgs.msg import ImuMsg
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import geometry_msgs.msg

IMU_PITCH_OFFSET = 0.0
IMU_ROLL_OFFSET = 0.0
IMU_YAW_OFFSET = 0.0

def reset_imu_offsets():
    global imu
    global IMU_PITCH_OFFSET
    global IMU_ROLL_OFFSET
    global IMU_YAW_OFFSET

    imu.update()

    IMU_PITCH_OFFSET = imu.pitch()
    IMU_ROLL_OFFSET = imu.roll()
    IMU_YAW_OFFSET = imu.yaw()

#bind all angles to -180 to 180
def clamp_angle_neg180_to_180(angle):
    angle_0_to_360 = clamp_angle_0_to_360(angle)
    if angle_0_to_360 > 180:
        return angle_0_to_360 - 180 * -1.0
    return angle_0_to_360
#bind all angles to -180 to 180
def clamp_angle_0_to_360(angle):
    return (angle + 1 * 360) - math.floor((angle + 2 * 360)/360)*360

def loop():
    global node
    global imu
    if imu.update():
        out_message = ImuMsg()
        
        # convert everything to a 0 to 360 to apply a 1d rotation then convert back to -180 to 180
        ROV_Pitch = clamp_angle_0_to_360(imu.roll()) - IMU_ROLL_OFFSET
        ROV_Roll = clamp_angle_0_to_360(imu.pitch()) - IMU_YAW_OFFSET
        ROV_Yaw = clamp_angle_0_to_360(imu.yaw()) - IMU_PITCH_OFFSET
        
        out_message.gyro = [ROV_Pitch, ROV_Roll, ROV_Yaw]

        ROV_X_Accel = imu.acceleration_x()
        ROV_Y_Accel = imu.acceleration_y()
        ROV_Z_Accel = imu.acceleration_z()
        out_message.accel = [ROV_X_Accel, ROV_Y_Accel, ROV_Z_Accel]

        pub.publish(out_message)


if __name__ == "__main__":
    global imu
    global node
    rclpy.init()
    node = rclpy.create_node('imu_sensor')
    
    imu = BNO055()
    time.sleep(1)
    pub = node.create_publisher(ImuMsg, 'imu', 1)
    
    timer = node.create_timer(1 / 20.0, loop)
    #reset_imu_offsets()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
