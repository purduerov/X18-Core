import lgpio as lg
import time

ACCEL_REPORT_ID = 0x04
ROTATION_REPORT_ID = 0x05

class BNO085():
    def __init__(self, i2c_bus, i2c_addr, i2c_flags):
        self.imu_handle = lg.i2c_open(i2c_bus, i2c_addr, i2c_flags)
        self.accel_data = [0.0, 0.0, 0.0] # x, y, z
        self.rotation_data = [0.0, 0.0, 0.0, 0.0] #x, y, z, w (quaternion)
        self.read_error = False
        self.is_setup = False

        self.sequence_number = 5 * [0]

    def read(self):
        (channel, payload) = self.shtp_rx()
        if(channel == -1):
            return False
        
        report_id = payload[0]

        if(report_id == ACCEL_REPORT_ID):
            self.accel_data[0] = self.parse_q14(payload[4:5])
            self.accel_data[1] = self.parse_q14(payload[6:8])
            self.accel_data[2] = self.parse_q14(payload[8:9])
        if(report_id == ROTATION_REPORT_ID):
            self.rotation_data[0] = self.parse_q14(payload[4:5])
            self.rotation_data[1] = self.parse_q14(payload[6:7])
            self.rotation_data[2] = self.parse_q14(payload[8:9])
            self.rotation_data[3] = self.parse_q14(payload[10:11])
        
        return True
    
    def shtp_tx(self, channel, payload : bytearray):
        length = len(payload) + 4
        tx_buf = payload
        split_length = bytearray([length & 0xFF, (length & 0xFF00) >> 8])


        tx_buf = split_length + bytearray([channel, self.sequence_number[channel]]) + tx_buf

        if (lg.i2c_write_device(self.imu_handle,  tx_buf) == length):
            self.sequence_number[channel] = (self.sequence_number[channel] + 1) & 0xFF
            return True
        else:
            return False
            
    
    def shtp_rx(self):
        (num_bytes, header) = lg.i2c_read_device(self.imu_handle, 4)
        if(num_bytes != 4):
            self.read_error = True
            return (-1, bytearray())
        
        packet_size = header[0] | (header[1] << 8) 
        packet_size &= 0x7FFF
        channel = header[3]

        packet_size -= 4
        
        (num_bytes, rx_buf) = lg.i2c_read_device(self.imu_handle, packet_size)

        if(num_bytes != packet_size):
            self.read_error = True
            return (-1, bytearray())
        else:
            self.read_error = False
        
        return (channel, rx_buf)
        

    
    def setup(self, report_rate):
        control = False
        while not control:
            (channel, message) = self.shtp_rx()
            if(channel == 0):
                control = True
    
        channel = 2
        accel_setup_payload = bytearray([
            0xFD,                 
            ACCEL_REPORT_ID,                  
            0x00,                  
            0x00,
            ((report_rate >> 0) & 0xFF) , ((report_rate >> 8) & 0xFF), 
            ((report_rate >> 16) & 0xFF), ((report_rate >> 24) & 0xFF), 
            0x00, 0x00, 0x00, 0x00 
        ])

        rotation_setup_payload = bytearray([
            0xFD,                 
            ROTATION_REPORT_ID,                  
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
    
    def parse_q14(self, bytes):
        raw_int = bytes[0] + bytes[1] << 8
        return raw_int / 16384.0

            
            
            
            