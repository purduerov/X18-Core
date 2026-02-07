import lgpio as lg
import time

I2C_BUS = 0
IMU_ADDR = 0x00



class BNO085():
    def __init__(self):
        self.imu_handle = lg.i2c_open(I2C_BUS, IMU_ADDR)
        MSG_SIZE = 10
        self.data = {
            "read_succes" : 0,
            "lin_accel" : {
                "x" : 0,
                "y" : 0,
                "z" : 0
            },
            "abs_orient" : {
                "x" : 0,
                "y" : 0,
                "z" : 0,
                "w" : 0
            },
            "temp" : 0
        }

        def get_packet(self):
            return 
        
        def read(self, packet : bytearray, reg : int):
            (rx_buf) = lg.i2c_block_process_call(self.handle, reg, packet)
            return self.data
        
        def setup(self):
            return