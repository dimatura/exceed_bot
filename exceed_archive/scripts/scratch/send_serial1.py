
import time
import atexit
import struct

from cobs import cobs
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)

def close_serial():
    ser.close()

ser.write(b'foo\n')
