import time
import atexit
import argparse
import serial

import logging
logging.basicConfig(format='%(asctime)s|%(name)s|%(levelname)s: %(message)s', level=logging.INFO)

def make_exit_handler(ser):
    def exit_handler():
        logging.info('closing port')
        ser.close()
    return exit_handler

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 115200)
    exit_handler = make_exit_handler(ser)
    atexit.register(exit_handler)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    logging.info('open: %r' % ser.is_open)
    time.sleep(0.2)

    ctr = 0

    parser = argparse.ArgumentParser()
    parser.add_argument('fname', type=str)
    args = parser.parse_args()
    with open(args.fname, 'w') as f:
        while True:
            ln = ser.read_until(b'\n')
            f.write(ln)
            ctr += 1
            if ctr % 20 == 0:
                logging.info('.')
