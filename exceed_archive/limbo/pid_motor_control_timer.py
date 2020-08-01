#!/usr/bin/env python

"""
version with encoder reading in its own timer
"""

import time
import sys
import os
import struct

import serial
from cobs import cobs
from simple_pid import PID

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Float64

import exceed_bot.maestro

MOTOR = 0
SERVO = 5

# won't move until ~1580
# note -- typical minmax values in RC are 988-2012, with 1500 being center
# subs range seems to be 172-1811
# TODO figure out weird reversal/break thing
# note here we use 1/4 pulses
MOTOR_CENTER = 1450*4
MOTOR_MIN = MOTOR_CENTER - 200*4
MOTOR_MAX = MOTOR_CENTER + 180*4

# TPS = ticks per s
# we use this to scale joystick input for setpoint
# no min - sadly the quadrature is not working, so we use as tachometer
MOTOR_TPS_MAX = 12.

# center between 1330-1500
SERVO_CENTER = 1409
# hard right 990-1113
SERVO_MIN = 1000
# hard left 1660-1880
SERVO_MAX = 1780

# teensy sends data every 50 ms
ENCODER_READ_PERIOD_S = 0.045

# not sure about time but seems to work ok
PID_SAMPLE_TIME_S = 0.01
PID_KP = 0.02
PID_KI = 0.02
PID_KD = 0.0


class Node(object):
    def __init__(self):
        rospy.init_node('pid_motor_control_node')

        self.ticks_per_s = 0.
        self.pid = PID(Kp=PID_KP, Ki=PID_KI, Kd=PID_KD, setpoint=0., sample_time=PID_SAMPLE_TIME_S, output_limits=(0.0, 1.0))

        # devices
        self.maestro = exceed_bot.maestro.Controller('/dev/maestro')
        self.encoder = serial.Serial('/dev/teensy', 115200)
        rospy.on_shutdown(self.shutdown)

        # ros
        self.encoder_timer = rospy.Timer(rospy.Duration(ENCODER_READ_PERIOD_S), self.read_encoder)
        self.pub_servo = rospy.Publisher('/servo_pulse', Float64, queue_size=10)
        self.pub_motor = rospy.Publisher('/motor_pulse', Float64, queue_size=10)
        self.sub_vel = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb)

    def read_encoder(self, event):
        # reading 8 bytes at 115200 bps should be about 0.5 ms
        # so plenty of time
        # TODO consider just reading this cmd_vel_cb - how frequent are joystick updates?
        # joystick update is ~44-64Hz (0.02-0.015 s)
        # so potentially setpoint changes more frequently than encoder reading
        buf = []
        for _ in range(256):
            b = self.encoder.read()
            if b == '\0':
                break
            buf.append(b)
        msg_data = cobs.decode(''.join(buf))
        if len(msg_data) != 4:
            rospy.logwarn('bad length: %d' % len(msg_data))
            return
        msg = struct.unpack('<I', msg_data)
        # not threadsafe but let's hope it's ok
        self.ticks_per_s = msg
        # update pid here?
        self.throttle_out = self.pid(self.ticks_per_s)

    def cmd_vel_cb(self, msg):
        steer = msg.angular.z
        steer_pulse = self.servo_map(steer)

        throttle_in = msg.linear.x
        # only using forward in this mode
        throttle_in_ticks = np.clip(throttle_in * MOTOR_TPS_MAX, 0.0, MOTOR_TPS_MAX)
        # throttle sets a setpoint, in ticks_per_s
        self.pid.setpoint = throttle_in_ticks

        # use encoder reading
        throttle_out = self.pid(self.ticks_per_s)
        # map back to pulse
        throttle_out_pulse = self.motor_map(throttle_out)

        self.pub_servo.publish(Float64(float(steer_pulse)))
        self.pub_motor.publish(Float64(float(throttle_out_pulse)))

        rospy.loginfo('steer %f %f', steer, steer_pulse)
        rospy.loginfo('throttle in: %f, in_ticks: %f, encoder_ticks: %f, pulse: %f',
                      throttle,
                      throttle_in_ticks,
                      self.ticks_per_s,
                      throttle_out_pulse)
        self.maestro.setTarget(SERVO, steer_pulse*4)
        # NOTE no x4 multiplier here
        self.maestro.setTarget(MOTOR, throttle_pulse)

    def servo_map(self, val):
        # -1, 1 to 0, 1
        val2 = (val + 1.0)*0.5
        val3 = SERVO_MIN*(1.0-val2) + val2*SERVO_MAX
        val4 = np.clip(val3, SERVO_MIN, SERVO_MAX)
        return int(val4)

    def motor_map(self, val):
        # -1, 1 to 0, 1
        val2 = (val + 1.0)*0.5
        val3 = MOTOR_MIN*(1.-val2) + val2*MOTOR_MAX
        val4 = np.clip(val3, MOTOR_MIN, MOTOR_MAX)
        return int(val4)

    def shutdown(self):
        rospy.loginfo('shutdown')
        self.maestro.stopAll()
        self.maestro.close()
        self.encoder.close()

    def run(self):
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            rospy.loginfo('hearbeat')
            rate.sleep()


if __name__ == '__main__':
    node = Node()
    node.run()
