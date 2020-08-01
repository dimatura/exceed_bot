
import numpy as np
import time
import atexit
import struct
from cobs import cobs
# from simple_pid import PID

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

target_ticks = 8

class Pid(object):
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.t0 = None
        self.last_output = None
        self.last_input = None
        self.proportional = 0.
        self.integral = 0.
        self.derivative = 0.

    def __call__(self, input_, setpoint):
        input_ = float(input_)
        t1 = time.time()
        if self.t0 is None:
            self.t0 = t1
            self.last_input = input_
            self.last_output = 0.0
            return self.last_output

        dt = t1 - self.t0
        print('dt: %f' % dt)

        error = setpoint - input_
        print('error: %f' %  error)
        d_input = input_ - self.last_input

        self.proportional = self.Kp * error

        self.integral += (self.Ki * error * dt)
        self.integral = np.clip(self.integral, -1.0, 1.0)
        self.derivative = -self.Kd * d_input/dt

        output = self.proportional + self.integral + self.derivative

        output = np.clip(output, -1.0, 1.0)

        self.last_output = output
        self.last_input = input_
        self.t0 = t1
        return output



pid = Pid(Kp=0.1, Ki=0.001, Kd=0.0)

throttle_pulse = MOTOR_CENTER


while True:
    ticks = read_msg()[0]
    if throttle_pulse >= MOTOR_CENTER:
        pass # positive
    else:
        # reversing
        ticks *= -1

    throttle = pid(ticks, 8.)
    throttle_pulse = motor_map(throttle)

    print('ticks: %f, throttle: %f' % (ticks, throttle_pulse/4.0))
    # note not multiplying by 4
    maestro.setTarget(MOTOR, int(throttle))
