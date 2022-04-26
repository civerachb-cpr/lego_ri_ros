#!/usr/bin/env python3
"""
SerialInterface.py

A simple tty-based interface to allow bi-directional communication with the Lego Mindstorms RI/Lego Spike Prime
hub.

The interface operates at 115200 baud 8N1 on the port specified.
"""

import serial
import rospy

from catkin.find_in_workspaces import find_in_workspaces as catkin_find

ctrl_c = b"\x03"
ctrl_e = b"\x05"
ctrl_d = b"\x04"

class SerialInterface:
    def __init__(self, port="/dev/lego", baud=115200):

        self.port = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

        # queued messages going each way
        self.tx_queue = []
        self.rx_queue = []

    def open(self):
        if self.port.isOpen():
            rospy.logdebug("port is already open")
            return
        try:
            self.port.open()
        except serial.SerialException as err:
            rospy.logerr(err)

    def close(self):
        if not self.port.isOpen():
            rospy.logdebug("port is already closed")
            return
        try:
            self.port.close()
        except Exception as err:
            rospy.logerr(err)

    def send_main(self):
        # to avoid serial corruption in paste-mode send the data one line at a time at 50 Hz
        rate = rospy.Rate(50)

        # cancel whatever's running first!
        for i in range(3):
            self.port.write(ctrl_c)
            rate.sleep()

        # skip past all the cruft during startup & the initial interpreter lines
        rospy.loginfo("Reading past boot messages...")
        l = self.port.readline()
        while not l.decode('utf-8').startswith('>>>'):
            l = self.port.readline()
            print(l)
        rospy.loginfo("Reading past boot interpreter prompt...")
        while l.decode('utf-8').startswith('>>>'):
            l = self.port.readline()
            print(l)

        path = catkin_find(project='lego_spike_interface', first_match_only=True, path='mindstorms/main.py')[0]
        rospy.loginfo('Sending Lego Hub main at {0}'.format(path))
        file_in = open(path, 'r')
        lines = file_in.readlines()
        file_in.close()
        self.port.write(ctrl_e)
        for l in lines:
            rospy.logdebug(l.rstrip('\n'))
            self.port.write(bytes(l, encoding='utf-8'))
            rate.sleep()
            self.port.readline()

        self.port.write(ctrl_d)
        rospy.loginfo('Lego Hub main sent!')

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            data = self.port.readline()
            rospy.loginfo(data)

            rate.sleep()

        self.port.write(ctrl_c)
