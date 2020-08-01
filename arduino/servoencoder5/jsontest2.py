import time
import atexit
import argparse

import logging
logging.basicConfig(format='%(asctime)s|%(name)s|%(levelname)s: %(message)s', level=logging.INFO)

import serial
import inputs
import json

fields = ['target_L', 'target_R', 'ticks_L', 'ticks_R', 'error_L', 'error_R', 'out_L', 'out_R']


def recv_msg(ser):
    s = ser.read_until(b'\n')
    #print(s)
    n = len(s)
    #logging.info('read %d bytes', n)
    if n > 0:
        return json.loads(s[:(n-1)])


def make_exit_handler(ser):
    def exit_handler():
        logging.info('closing port')
        ser.close()
    return exit_handler


def main():
    for device in inputs.devices.gamepads:
        print(device)

    #ser = serial.Serial('/dev/ttyACM0', 9600)
    ser = serial.Serial('/dev/ttyACM0', 115200)
    #ser = serial.Serial('/dev/zumo', 115200)
    #ser = serial.Serial('/dev/ttyUSB0', 115200)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    exit_handler = make_exit_handler(ser)
    atexit.register(exit_handler)
    logging.info('open: %r' % ser.is_open)
    logging.info(ser.writable())

    time.sleep(1.0)
    #cmd = {"target_ticks_per_s": 0, "target_steer_deg": 90, "kp": 6e-4, "kd": 0, "ki": 0.0000}
    #msg = json.dumps(cmd) + '\n'
    #wrote = ser.write(msg.encode())
    #time.sleep(0.5)

    cmd = ["ping", {}]
    msg = json.dumps(cmd) + '\n'
    wrote = ser.write(msg.encode())
    recv = recv_msg(ser)

    # 0.
    cmd = ["pid", {"kp": 0.0, "kd": 0.0, "ki": 0.0001, "kf": 0.001}]
    msg = json.dumps(cmd) + '\n'
    wrote = ser.write(msg.encode())
    recv = recv_msg(ser)
    print(recv)

    while True:
        #cmd = {"target_ticks_per_s": 140, "target_steer_deg": 90}
        cmd = ["motion", {"target_ticks_per_s": 180, "steer_input_deg": 90}]
        msg = json.dumps(cmd) + '\n'
        wrote = ser.write(msg.encode())
        # time.sleep(0.1)
        recv = recv_msg(ser)
        #logging.info(recv)
        formatted_msg = ', '.join(["{}:{:8.2f}".format(k, v) for (k, v) in recv.items()])
        logging.info(formatted_msg)
        time.sleep(0.2)


if __name__ == "__main__":
    main()
