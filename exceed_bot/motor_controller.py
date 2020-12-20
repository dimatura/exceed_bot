import time
import atexit
import argparse
import json

import serial
import rospy


def _make_exit_handler(ser):
    def exit_handler():
        ser.close()
    return exit_handler


class MotorController:
    def __init__(self, tty_url='/dev/ttyACM0'):
        self.tty_url = tty_url
        try:
            self.ser = serial.Serial(self.tty_url, 115200, timeout=1.0)
        except serial.serialutil.SerialException as ex:
            rospy.logerr("serial not found: %r" % ex)
            self.ser = None
            return
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        rospy.loginfo('serial %s open: %r' % (self.tty_url, self.ser.is_open))

    def close(self):
        if self.ser is None:
            return
        self.ser.close()

    def register_atexit_closer(self):
        atexit.register(_make_exit_handler(self.ser))

    def _send(self, msg, get_reply=True):
        if self.ser is None:
            rospy.logwarn("self.ser is None")
            return
        msg = json.dumps(msg) + '\n'
        wrote = self.ser.write(msg.encode())
        if get_reply:
            reply = self._receive()
            return reply
        else:
            return wrote

    def _receive(self):
        if self.ser is None:
            return
        s = self.ser.read_until(b'\n')
        n = len(s)
        # print('received %d' %  n)
        if n > 0:
            try:
                decoded = json.loads(s[:(n-1)])
            except json.decoder.JSONDecodeError:
                rospy.logerr("Error decoding JSON reply from motor controller")
                decoded = None
            return decoded
        else:
            return None

    def ping(self):
        reply = self._send(["ping", {}])
        return reply

    def set_pid(self, kp, ki, kd, kf):
        reply = self._send(["pid", {"kp": kp, "kd": kd, "ki": ki, "kf": kf}])
        return reply

    def motor_cmd(self, ticks_per_s, steer_deg):
        reply = self._send(["motion", {"target_ticks_per_s": ticks_per_s, "steer_input_deg": steer_deg}])
        return reply
