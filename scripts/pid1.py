
import time
import atexit
import struct
from cobs import cobs

import serial

import exceed_bot.maestro

MOTOR = 0
MOTOR_CENTER = 1450*4
MOTOR_MIN = MOTOR_CENTER - 200*4
MOTOR_MAX = MOTOR_CENTER + 200*4


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
    print val3
    val4 = min(max(val3, MOTOR_MIN), MOTOR_MAX)
    return int(val4)


atexit.register(close_serial)

#ser = serial.Serial('/dev/ttyACM0', 115200)
#ser = serial.Serial('/dev/ttyACM1', 115200)
ser = serial.Serial('/dev/ttyACM2', 115200)
print(ser.is_open)
maestro = exceed_bot.maestro.Controller('/dev/ttyACM0')

target_ticks = 8

throttle_pulse = MOTOR_CENTER

while True:
    ticks = read_msg()[0]
    print(ticks)
    #throttle_pulse = motor_map(0.45)
    if ticks < target_ticks:
        throttle_pulse += 1
    elif ticks > target_ticks:
        throttle_pulse -= 1
    else:
        pass

    throttle_pulse = min(max(throttle_pulse, MOTOR_MIN), MOTOR_MAX)

    print(throttle_pulse/4.0)
    # note not multiplying by 4
    maestro.setTarget(MOTOR, throttle_pulse)
