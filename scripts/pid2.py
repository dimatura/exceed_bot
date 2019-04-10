
import numpy as np
import time
import atexit
import struct
from cobs import cobs
from simple_pid import PID

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
    val4 = np.clip(val3, MOTOR_MIN, MOTOR_MAX)
    return int(val4)



atexit.register(close_serial)

#ser = serial.Serial('/dev/ttyACM0', 115200)
#ser = serial.Serial('/dev/ttyACM1', 115200)
ser = serial.Serial('/dev/teensy', 115200)
print(ser.is_open)
#maestro = exceed_bot.maestro.Controller('/dev/ttyACM0')
maestro = exceed_bot.maestro.Controller('/dev/maestro')

target_ticks = 8.0

pid = PID(Kp=0.02, Ki=0.02, Kd=0.0, setpoint=target_ticks, sample_time=0.01, output_limits=(0.0, 1.0))


print('calibration')
maestro.setTarget(MOTOR, MOTOR_CENTER)
time.sleep(1.0)


tic = time.time()

while True:
    ticks = read_msg()[0]
    err = target_ticks - ticks

    throttle = pid(float(ticks))
    throttle_pulse = motor_map(throttle)

    print(pid.setpoint, err, ticks, throttle, throttle_pulse/4.0)

    # note not multiplying by 4
    maestro.setTarget(MOTOR, int(throttle_pulse))

    elapsed = time.time() - tic
    if elapsed > 2.0:
        pid.setpoint = 0.0
    if elapsed > 4.0:
        pid.setpoint = 10.0
    if elapsed > 6.0:
        pid.setpoint = 4.0
    if elapsed > 8.0:
        pid.setpoint = 12.0
    if elapsed > 10.0:
        pid.setpoint = 8.0
