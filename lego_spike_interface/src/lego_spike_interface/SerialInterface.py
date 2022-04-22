#!/usr/bin/env python3
"""
SerialInterface.py

A simple tty-based interface to allow bi-directional communication with the Lego Mindstorms RI/Lego Spike Prime
hub.

The interface operates at 115200 baud 8N1 on the port specified.
"""

import serial
import rospy

class SerialInterface:
    def __init__(self, port="/dev/lego", baud=115200):
        self.ctrl_c = "\x03"

        self.port = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
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
