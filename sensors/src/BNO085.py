import lgpio as lg
import time

class BNO085():
    def __init__(self, i2c_bus, i2c_addr, i2c_flags):
        self.imu_handle = lg.i2c_open(i2c_bus, i2c_addr, i2c_flags)
        self.accel_data = tuple(3 * [0])
        self.rotation_data = tuple(4 * [0])
        self.read_error = False
        self.is_setup = False

        self.sequence_number = 5 * [0]

        self.ACCEL_REPORT_ID = 0x01
        self.ROTATION_REPORT_ID = 0x05

    def read(self):

        return 
    
    def shtp_tx(self, channel, payload : bytearray):
        length = len(payload) + 4
        tx_buf = payload
        split_length = bytearray([(length & 0xFF00) >> 8, length & 0xFF])


        tx_buf = split_length + bytearray([channel, self.sequence_number[channel]]) + tx_buf

        return (lg.i2c_write_device(self.imu_handle,  tx_buf) == length)
    
    def shtp_rx(self):
        (num_bytes, header) = lg.i2c_read_device(self.imu_handle, 4)
        if(num_bytes != 4):
            self.read_error = False
            return bytearray()
        
        packet_size = header[0] | (header[1] << 8)
        
        (num_bytes, rx_buf) = lg.i2c_read_device(self.imu_handle, packet_size)

        if(num_bytes != packet_size):
            self.read_error = False
        
        return rx_buf
        

    
    def setup(self, report_rate):
        channel = 2
        accel_setup_payload = bytearray([
            0xFD,                 
            self.ACCEL_REPORT_ID,                  
            0x00,                  
            0x00,
            ((report_rate >> 0) & 0xFF) , ((report_rate >> 8) & 0xFF), 
            ((report_rate >> 16) & 0xFF), ((report_rate >> 24) & 0xFF), 
            0x00, 0x00, 0x00, 0x00 
        ])

        rotation_setup_payload = bytearray([
            0xFD,                 
            self.ROTATION_REPORT_ID,                  
            0x00,                  
            0x00,
            ((report_rate >> 0) & 0xFF) , ((report_rate >> 8) & 0xFF), 
            ((report_rate >> 16) & 0xFF), ((report_rate >> 24) & 0xFF), 
            0x00, 0x00, 0x00, 0x00 
        ])
        

        self.is_setup = self.shtp_tx(channel, accel_setup_payload) and self.shtp_tx(channel, rotation_setup_payload)
        return

    def close(self):
        lg.i2c_close(self.imu_handle)
            
            
            
            