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

        return True

    def read_temperature(self):
        if self._handle < 0:
            print("No bus!")
            return False

        self._temperature = lg.i2c_read_i2c_block_data(self._handle, self._ADT7410_TEMP_ADDR, 2)
        self._temperature[1] = (self._temperature[0] << 5) | (self._temperature[1] >> 3)
        self._temperature[0] = self._temperature[0] >> 3        