#! /usr/bin/python

import signal
import sys
import time

import can

N_CAN_BYTES = 8
ZERO_THROTTLE = 127


def getSignal(bus):
    def signal_handler(sig, frame) -> None:
        print("CTRL+C detected")
        zeroOutThrusters(bus=bus)
        print("Thrusters zero-ed out")
        sys.exit(0)

    return signal_handler


def zeroOutThrusters(bus=None) -> None:
    a = mapThrusters([ZERO_THROTTLE] * N_CAN_BYTES)

    writeToCan(a, timesleep=0.0, bus=bus, printOut=True)


def mapThrusters(can_pow, can_map=None, printOut=False) -> dict:
    if can_map is None:
        can_map = {
            0x201: [7, 0, 0, 0],
            0x202: [0, 4, 5, 6]
        }

    can_out = {}

    for cid in can_map:
        data = [140, 140, 140, 140]
        cur = can_map[cid]

        for el in cur:
            if el is not None:
                data.append(can_pow[el])
            else:
                data.append(127)

        if printOut:
            print("|{}|".format(cid))
            for el in data:
                print("{}".format(el))
            print("----")

        can_out[cid] = data

    return can_out


def writeToCan(packet, timesleep=1, bus=None, printOut=False) -> None:
    if bus is None:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')

    for cid in packet:
        data = bytearray(packet[cid])

        can_tx = can.Message(arbitration_id=cid, data=data,is_extended_id=False)

        bus.send(can_tx, timeout=1)

        if printOut:
            tst = f"    0x{cid:x}:"
            for el in data:
                tst += f" {int(el):03}"

            print(tst)


def mainLoop(timesleep=1, bound=5, increment=1, mid=127, channel='can0', bustype='socketcan'):
    can_bus = can.interface.Bus(channel=channel, bustype=bustype)

    signal.signal(signal.SIGINT, getSignal(can_bus))

    inc = increment
    offset = 0
    while True:
        num = 127 + offset
        print("Thrusters setting to {}".format(num))

        base = [num] * 8

        thrusts = mapThrusters(base)

        writeToCan(thrusts, timesleep=timesleep, bus=can_bus, printOut=True)

        if offset >= bound:
            inc = -increment
        elif offset <= -bound:
            inc = increment

        offset += inc

        time.sleep(timesleep)

    zeroOutThrusters(bus=can_bus)


if __name__ == "__main__":
    bound = 10 * 10
    inc = 2
    print(mainLoop(bound=bound, increment=inc, timesleep=0.1))