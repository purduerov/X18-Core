#External Temperature Sensor (ADT7410)

import lgpio as lg
from time import sleep



class ADT7410(object):
    #Registers
    _ADT7410_ADDR = 0x48 #CHECK ACTUAL VALUE
    _ADT7410_TEMP_ADDR = 0x0 #0x1 is LSB
    _ADT7410_RESET = 0x2F

    def __init__(self, bus =1): 
        try:
            self._handle = lg.i2c_open(bus, self._AD7410_ADDR, 0)
        except:
            print("Bus %d is not available." % bus)
            print("Available busses are listed as /dev/i2c*")
            self._bus = None
        
        self._temperature = 0
    
    def init(self):
        if self._handle < 0:
            "No bus!"
            return False

        # self._bus.write_byte(self._MS5837_ADDR, self._MS5837_RESET)
        write_byte = lg.i2c_write_byte(self._handle, self._ADT7410_RESET)
        # Wait for reset to complete
        sleep(0.01)

        self.data = []
        #data = i2c_read_word_data(self.handle, self._ADT7410_TEMP_ADDR)

        self.data[1] = lg.i2c_read_byte_data(self._handle, self._ADT7410_TEMP_ADDR)
        self.data[0] = lg.i2c_read_byte_data(self._handle, self._ADT7410_TEMP_ADDR + 1)


        #still need to send the data to the hat_temp file
        msb = self.data[1]
        lsb = self.data[0]
        temp_combined = (msb << 8 | (lsb & 0xFF)) >> 3

        if temp_combined > 4095:
            temp_combined -= 8192

        temperature_c = temp_combined / 16.0