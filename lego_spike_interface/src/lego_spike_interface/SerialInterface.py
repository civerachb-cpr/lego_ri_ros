#!/usr/bin/env python3
"""
SerialInterface.py

A simple tty-based interface to allow bi-directional communication with the Lego Mindstorms RI/Lego Spike Prime
hub.

The interface operates at 115200 baud 8N1 on the port specified.
"""

import serial
import rospy

from lego_spike_msgs.msg import Color
from lego_spike_msgs.msg import ColorSensors
from lego_spike_msgs.msg import DistanceSensors
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float32

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

        # TODO parameterize all these
        self.imu_frame_id = 'lego_hub_imu_link'


        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.color_sensors_pub = rospy.Publisher('colors', ColorSensors, queue_size=1)
        self.distance_sensors_pub = rospy.Publisher('distance', DistanceSensors, queue_size=1)


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
        # our main sensor-reading loop runs at 10Hz
        # TODO: can we go faster?
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            data = self.port.readline().decode('utf-8')
            try:
                d = eval(data)
                self.send_msgs(d)
            except Exception as err:
                rospy.logerr(err)
                rospy.logerr(data)
            rate.sleep()

        # cancel when we're done
        self.port.write(ctrl_c)

    def send_msgs(self, data):
        hdr = Header()
        hdr.stamp = rospy.Time.now()
        hdr.frame_id = self.imu_frame_id
        imu_msg = Imu()
        imu_msg.header = hdr
        imu_msg.angular_velocity.x = data['imu']['angular']['x']
        imu_msg.angular_velocity.y = data['imu']['angular']['y']
        imu_msg.angular_velocity.z = data['imu']['angular']['z']
        imu_msg.linear_acceleration.x = data['imu']['linear']['x']
        imu_msg.linear_acceleration.y = data['imu']['linear']['y']
        imu_msg.linear_acceleration.z = data['imu']['linear']['z']

        hdr = Header()
        hdr.stamp = rospy.Time.now()
        js = JointState()
        js.header = hdr
        js.name = []
        js.position = []
        js.effort = []
        js.velocity = []
        clr = ColorSensors()
        clr.name = []
        dst = DistanceSensors()
        dst.name = []
        dst.data = []

        for device in data['devices']:
            if device['type'] == 'motor':
                js.name.append('motor_{0}_wheel_link'.format(device['port']))
                js.position.append(device['data']['position'])
                js.velocity.append(device['data']['speed'])
                js.effort.append(0.0)  # TODO?
            elif device['type'] == 'color':
                clr.name.append('color_{0}'.format(device['port']))
                c = Color()
                c.brightness = device['data']['level']
                c.r = device['data']['rgb']['r']
                c.g = device['data']['rgb']['g']
                c.b = device['data']['rgb']['b']
                clr.data.append(c)
            elif device['type'] == 'distance':
                dst.name.append('distance_{0}'.format(device['port']))
                dst.data.append(device['data'])

        self.imu_pub.publish(imu_msg)
        self.joint_state_pub.publish(js)
        self.color_sensors_pub.publish(clr)
        self.distance_sensors_pub.publish(dst)
