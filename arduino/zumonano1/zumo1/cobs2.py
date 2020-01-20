import time
import atexit
import argparse

import logging
logging.basicConfig(format='%(asctime)s|%(name)s|%(levelname)s: %(message)s', level=logging.INFO)

import struct
import serial
import inputs
from cobs import cobs

fields = ['target_L', 'target_R', 'ticks_L', 'ticks_R', 'error_L', 'error_R', 'out_L', 'out_R']

def recv_msg2(ser):
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


def main():
    for device in inputs.devices.gamepads:
        print(device)

    #ser = serial.Serial('/dev/ttyACM0', 9600)
    #ser = serial.Serial('/dev/ttyACM0', 115200)
    ser = serial.Serial('/dev/zumo', 115200)
    #ser = serial.Serial('/dev/ttyUSB0', 115200)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    atexit.register(exit_handler)
    logging.info('open: %r' % ser.is_open)
    logging.info(ser.writable())

    left = 0
    right = 0
    while True:

        events = inputs.get_gamepad()
        for event in events:
            #print('type: {}, code: {}, state: {}'.format(event.ev_type, event.code, event.state))
            if event.code == 'ABS_Y':
                left = -float(event.state)/20.0
                print('l: %f' % left)
            elif event.code == 'ABS_RY':
                right = -float(event.state)/20.0
                print('r: %f' % right)

        #msg0 = struct.pack('ff', 2., 0.0)
        #msg1 = cobs.encode(msg0)
        #msg2 = cobs.encode(msg1) + '\x00'
        ##logging.info('writing')
        ##logging.info(msg)
        #wrote = ser.write(msg2)
        ##logging.info('wrote %d bytes', wrote)

        msg0 = struct.pack('ff', float(left), float(right))
        msg1 = cobs.encode(msg0) + b'\x00'
        logging.info('writing')
        #logging.info(msg)
        wrote = ser.write(msg1)
        logging.info('wrote %d bytes', wrote)

        # get ack
        #logging.info('reading')
        recv = recv_msg2(ser)

        msg_parts = [ ('%s: %06.2f' % (fld, val)) for (fld, val) in zip(fields, recv)]
        msg = ', '.join(msg_parts)

        logging.info(msg)
        #time.sleep(1.)


if __name__ == "__main__":
    main()
