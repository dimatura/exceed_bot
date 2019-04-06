
import time
import atexit
import struct
from cobs import cobs

import serial

def close_serial():
    ser.close()

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

atexit.register(close_serial)

ser = serial.Serial('/dev/ttyACM0', 115200)


while True:
    msg = read_msg()
    print(msg)

