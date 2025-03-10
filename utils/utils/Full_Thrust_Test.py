#! /usr/bin/python3

import signal
import sys
import time

from spidev import SpiDev

import numpy as np
spi = SpiDev()
spi.open(0,1)
spi.max_speed_hz = 50000
OFF_THRUST = 127


def signal_handler(sig, frame) -> None:
    print("CTRL+C detected")
    zeroOutThrusters()
    print("Thrusters zero-ed out")
    sys.exit(0)

def zeroOutThrusters() -> None:
    thrusts = [OFF_THRUST] * 8
    thrusts = bytearray(thrusts)
    writeSPI(thrusts, printOut=True)

def writeSPI(packet, printOut=True) -> None:
    # publish = bytearray(packet)
    publish = bytearray(packet)
    if printOut:
        print(publish)
    spi.writebytes(publish)

def mainLoop(timesleep=1, bound=5, increment=1):
    
    signal.signal(signal.SIGINT, signal_handler)

    inc = increment
    offset = 1
    num = 127
    while True:
        num = num + offset
        print("Thrusters numsetting to {}".format(num))

        thrusts = [num] * 8
        #thrusts = [127, 127,127,127,127,num,127,127]
        
        writeSPI(thrusts)

        if offset >= 0 and (num + offset) < 255:
            offset = 1
        elif (num + offset >= 256):
            offset = -1
        elif (num + offset <= -1):
            offset = 1
        elif offset <= 0 and (num + offset) >= 0:
            offset = -1

        time.sleep(timesleep)

    zeroOutThrusters()

if __name__ == "__main__":
    bound = 127
    inc = 20
    print(mainLoop(bound=bound, increment=inc, timesleep=(1.0/50)))
