#NEW IMU 
#Datasheets https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
#Register Map https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

import lgpio as lg
import time


class MPU6050 ():

    CONFIG = 0x1A
    GYROCONFIG = 0x1B
    ACCELCONFIG = 0x1C
    RAW_DATA = 0x3B

    def __init__ (self, i2c_bus, i2c_addr, i2c_flags):
        self.imu_handle = lg.i2c_open(i2c_bus, i2c_addr, i2c_flags)
        self.accel_data = [0.0, 0.0, 0.0] # x, y, z
        self.gyro_data = [0.0, 0.0, 0.0] #x, y, z
        self.read_error = False
        self.write_error = False
    
    def setup(self):
        self.write_error = False
        invalid_write += lg.i2c_write_byte_data(self.imu_handle, self.CONFIG, 0x00)
        invalid_write += lg.i2c_write_byte_data(self.imu_handle, self.GYROCONFIG, 0x08)
        invalid_write += lg.i2c_write_byte_data(self.imu_handle, self.ACCELCONFIG, 0x08)
        if invalid_write:
            self.write_error = True

        
    def read_imu(self):
        self.read_error = False
        (count, raw) = lg.i2c_read_i2c_block_data(self.handle, self.RAW_DATA, 14)
        if count != 14:
            self.read_error = True
            return
        
        self.accel_data[0] = (raw[0] << 8) + raw[1]
        self.accel_data[1] = (raw[2] << 8) + raw[3]
        self.accel_data[2] = (raw[4] << 8) + raw[5]
        
        self.gyro_data[0] = (raw[8] << 8) + raw[9]
        self.gyro_data[1] = (raw[10] << 8) + raw[11]
        self.gyro_data[2] = (raw[12] << 8) + raw[13]
        