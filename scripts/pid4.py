
import numpy as np
import time
import atexit
import struct
from cobs import cobs

import serial

import exceed_bot.maestro

MOTOR = 0
MOTOR_CENTER = 1450*4
MOTOR_MIN = MOTOR_CENTER - 180*4
MOTOR_MAX = MOTOR_CENTER + 200*4
print(MOTOR_MIN/4.0, MOTOR_MAX/4.0)


def close_serial():
    ser.close()
    maestro.stopAll()
    maestro.close()


def read_msg():
    buf = []
    while True:
        b = ser.read()
        if b == '\0':
            break
        buf.append(b)
    msg_data = cobs.decode(''.join(buf))
    if len(msg_data) != 4:
        print('bad length: %d' % len(msg_data))
        return 0
    msg = struct.unpack('<I', msg_data)
    return msg


def motor_map(val):
    # -1, 1 to 0, 1
    val2 = (val + 1.0)/2.
    val3 = MOTOR_MIN*(1.-val2) + val2*MOTOR_MAX
    val4 = np.clip(val3, MOTOR_MIN, MOTOR_MAX)
    return int(val4)



atexit.register(close_serial)

#ser = serial.Serial('/dev/ttyACM0', 115200)
#ser = serial.Serial('/dev/ttyACM1', 115200)
ser = serial.Serial('/dev/teensy', 115200)
print(ser.is_open)
#maestro = exceed_bot.maestro.Controller('/dev/ttyACM0')
maestro = exceed_bot.maestro.Controller('/dev/maestro')

target_ticks = 3.0

throttle0 = 0.0
throttle1 = 0.0
err0 = 0.0

Kp = 0.005
#Kp = 0.01

while True:
    ticks = read_msg()[0]
    #if throttle0 < -0.2:
    #    ticks = -ticks

    err = float(target_ticks - ticks)
    print('err: %d' % err)
    correction = Kp*err

    throttle1 = throttle0 + correction
    throttle1 = np.clip(throttle1, -1.0, 1.0)

    throttle_pulse = motor_map(throttle1)

    print(ticks, throttle1, throttle_pulse/4.0)
    # note not multiplying by 4
    maestro.setTarget(MOTOR, int(throttle_pulse))
    throttle0 = throttle1
