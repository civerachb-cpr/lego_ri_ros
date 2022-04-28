#!/usr/bin/env python3
"""
Commands sent over the serial interface to the Lego hub
"""

import rospy
from threading import Lock

class CommandList:
    ACTION_LIGHTS='lights'
    ACTION_MOTORS='motors'
    ACTION_MOTOR_CONFIG='motorcfg'

    def __init__(self, actions=[], parameters=[]):
        self.actions = actions
        self.parameters = parameters
        self.mutex = Lock()

    def __str__(self):
        if len(self) > 0:
            return "{{ 'actions': {0}, 'parameters': {1} }}".format(self.actions, self.parameters)
        else:
            return ''

    def __len__(self):
        return len(self.actions)

    def append(self, action, parameter):
        self.mutex.acquire()
        self.actions.append(action)
        self.parameters.append(parameter)
        self.mutex.release()

    def clear(self):
        self.mutex.acquire()
        self.actions = []
        self.parameters = []
        self.mutex.release()

    def transmit(self, interface):
        self.mutex.acquire()
        if len(self) > 0:
            rospy.logwarn(self)
            interface.write_line(str(self))
            # clear, but we already have the lock!
            self.actions = []
            self.parameters = []
        self.mutex.release()
