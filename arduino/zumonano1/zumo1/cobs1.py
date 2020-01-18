import time
import atexit

import logging
logging.basicConfig(format='%(asctime)s|%(name)s|%(levelname)s: %(message)s', level=logging.INFO)

import struct
import serial
from cobs import cobs

fields = ['target_L', 'target_R', 'ticks_L', 'ticks_R', 'error_L', 'error_R', 'out_L', 'out_R']

def recv_msg2():
    s = ser.read_until(b'\x00')
    n = len(s)
    logging.info('read %d bytes', n)
    if n > 0:
        decoded0 = cobs.decode(s[:(n-1)])
        n_bin = len(decoded0)
        #logging.info('decoded to %d bytes', n_bin)
        #decoded1 = struct.unpack('ffffffffff', decoded0)
        decoded1 = struct.unpack('f'*8, decoded0)
        return decoded1


def exit_handler():
    logging.info('closing port')
    ser.close()

#ser = serial.Serial('/dev/ttyACM0', 9600)
ser = serial.Serial('/dev/ttyACM0', 115200)
#ser = serial.Serial('/dev/ttyUSB0', 115200)
ser.reset_input_buffer()
ser.reset_output_buffer()
atexit.register(exit_handler)
logging.info('open: %r' % ser.is_open)
logging.info(ser.writable())

while True:

    #msg0 = struct.pack('ff', 2., 0.0)
    #msg1 = cobs.encode(msg0)
    #msg2 = cobs.encode(msg1) + '\x00'
    ##logging.info('writing')
    ##logging.info(msg)
    #wrote = ser.write(msg2)
    ##logging.info('wrote %d bytes', wrote)

    msg0 = struct.pack('ff', 200.0, 200.0)
    msg1 = cobs.encode(msg0) + b'\x00'
    logging.info('writing')
    #logging.info(msg)
    wrote = ser.write(msg1)
    logging.info('wrote %d bytes', wrote)

    # get ack
    #logging.info('reading')
    recv = recv_msg2()

    msg_parts = [ ('%s: %06.2f' % (fld, val)) for (fld, val) in zip(fields, recv)]
    msg = ', '.join(msg_parts)

    logging.info(msg)
    #time.sleep(1.)
