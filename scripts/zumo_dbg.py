import time
import atexit

import logging
logging.basicConfig(format='%(asctime)s|%(name)s|%(levelname)s: %(message)s', level=logging.INFO)

import struct
import serial
from cobs import cobs


def recv_msg2():
    s = ser.read_until(b'\x00')
    n = len(s)
    #logging.info('read %d bytes', n)
    if n > 0:
        decoded0 = cobs.decode(s[:(n-1)])
        n_bin = len(decoded0)
        #logging.info('decoded to %d bytes', n_bin)
        decoded1 = struct.unpack('ffffffff', decoded0)
        return decoded1


def exit_handler():
    logging.info('closing port')
    ser.close()

ser = serial.Serial('/dev/ttyACM0', 115200)
ser.reset_input_buffer()
ser.reset_output_buffer()
atexit.register(exit_handler)
logging.info('open: %r' % ser.is_open)
logging.info(ser.writable())


tic = time.time()
while True:
    #msg0 = struct.pack('ff', 0., 0.)
    #msg0 = struct.pack('ff', 200, 200.)
    msg0 = struct.pack('ff', 400, 400.)
    msg1 = cobs.encode(msg0) + '\x00'
    wrote = ser.write(msg1)

    # get ack
    recv = recv_msg2()
    logging.info(recv)

    #if time.time() - tic > 2.0:
    #    msg0 = struct.pack('ff', 0., 0.)
    #    msg1 = cobs.encode(msg0) + '\x00'
    #    wrote = ser.write(msg1)
