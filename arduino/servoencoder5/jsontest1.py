import time
import atexit
import argparse

import logging
logging.basicConfig(format='%(asctime)s|%(name)s|%(levelname)s: %(message)s', level=logging.INFO)

import struct
import serial
import inputs
import json

fields = ['target_L', 'target_R', 'ticks_L', 'ticks_R', 'error_L', 'error_R', 'out_L', 'out_R']

def make_exit_handler(ser):
    def exit_handler():
        logging.info('closing port')
        ser.close()
    return exit_handler


class CommWrapper:
    def __init__(self):
        #ser = serial.Serial('/dev/ttyACM0', 9600)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        #ser = serial.Serial('/dev/zumo', 115200)
        #ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        atexit.register(make_exit_handler(self.ser))
        logging.info('open: %r' % self.ser.is_open)
        logging.info(self.ser.writable())

    def send(self, msg, get_reply=True):
        msg = json.dumps(msg) + '\n'
        wrote = self.ser.write(msg.encode())
        if get_reply:
            reply = self.receive()
            return reply
        else:
            return wrote

    def receive(self):
        s = self.ser.read_until(b'\n')
        n = len(s)
        if n > 0:
            return json.loads(s[:(n-1)])
        else:
            return None


def main():
    comms = CommWrapper()

    for device in inputs.devices.gamepads:
        print(device)

    time.sleep(1.0)

    cmd = ["ping", {}]
    pong = comms.send(cmd)

    #cmd = ["pid", {"kp": 0.0001, "kd": 0.0, "ki": 0.0006, "kf": 0.001}]
    #pid_reply = comms.send(cmd)

    time.sleep(0.5)

    if False:
        while True:
            #cmd = {"target_ticks_per_s": 140, "target_steer_deg": 90}
            cmd = {"target_ticks_per_s": 0, "target_steer_deg": 90}
            msg = json.dumps(cmd) + '\n'
            wrote = ser.write(msg.encode())
            time.sleep(0.1)
            recv = recv_msg(ser)
            #logging.info(recv)
            formatted_msg = ', '.join(["{}:{:8.2f}".format(k, v) for (k, v) in recv.items()])
            logging.info(formatted_msg)
            time.sleep(0.1)

    steer = 90
    throttle = 0
    while True:
        events = inputs.get_gamepad()
        # input lib seems inconsistent in interpreting joystick :/
        if False:
            for event in events:
                print('type: {}, code: {}, state: {}'.format(event.ev_type, event.code, event.state))
                if event.code == 'ABS_Y':
                    steer = -float(event.state)/20.0
                    print('l: %f' % steer)
                elif event.code == 'ABS_RY':
                    throttle = -float(event.state)/20.0
                    print('r: %f' % throttle)
        else:
             for event in events:
                #print('type: {}, code: {}, state: {}'.format(event.ev_type, event.code, event.state))
                if event.code == 'ABS_X':
                    norm_steer = (float(event.state) - 128)/256.0
                    steer = (norm_steer*180 + 90)
                    print('steer: %f' % steer)
                elif event.code == 'ABS_RZ':
                    norm_throttle = -2.0*(float(event.state) - 128)/256.0
                    if abs(norm_throttle) < 0.1:
                        throttle = 0
                    else:
                        throttle = 200*norm_throttle
                    print('throttle: %f' % throttle)


        cmd = ["motion", {"target_ticks_per_s": throttle, "steer_input_deg": steer}]
        motion_reply = comms.send(cmd)
        formatted_msg = ', '.join(["{}:{:8.2f}".format(k, v) for (k, v) in motion_reply.items()])
        logging.info(formatted_msg)
        #time.sleep(1.)


if __name__ == "__main__":
    main()
