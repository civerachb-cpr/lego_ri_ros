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
from lego_spike_msgs.msg import LightPattern
from lego_spike_msgs.msg import MotorCfg
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float32

from lego_spike_interface.command import CommandList

from catkin.find_in_workspaces import find_in_workspaces as catkin_find

ctrl_c = b"\x03"
ctrl_e = b"\x05"
ctrl_d = b"\x04"

def goal_pos_callback(data, interface):
    interface.set_goal_positions(data)

def light_pattern_callback(data, interface):
    interface.set_lights(data)

def motor_cfg_callback(data, interface):
    interface.set_motor_config(data)

class LegoInterface:
    def __init__(self):
        # TODO parameterize all these
        self.imu_frame_id = 'lego_hub_imu_link'

        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)
        self.temperature_pub = rospy.Publisher('temperature', Float32, queue_size=1)
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.color_sensors_pub = rospy.Publisher('colors', ColorSensors, queue_size=1)
        self.distance_sensors_pub = rospy.Publisher('distance', DistanceSensors, queue_size=1)

        self.goal_pos_sub = rospy.Subscriber('cmd/goal_position', JointState, goal_pos_callback, self)
        self.motor_cfg_sub = rospy.Subscriber('cmd/motor_config', MotorCfg, motor_cfg_callback, self)
        self.light_pattern_sub = rospy.Subscriber('cmd/lights', LightPattern, light_pattern_callback, self)

        self.command_queue = CommandList()

    def send_main(self, path=None):
        "Sends main.py to the Lego Hub so it can communicate bidirectionally"

        # to avoid serial corruption in paste-mode send the data one line at a time at 50 Hz
        rate = rospy.Rate(50)

        # cancel whatever's running first!
        for i in range(3):
            self.write_byte(ctrl_c)
            rate.sleep()

        # skip past all the cruft during startup & the initial interpreter lines
        rospy.loginfo("Reading past boot messages...")
        l = self.read_line()
        while not l.startswith('>>>'):
            l = self.read_line()
            rospy.logdebug(l)
        rospy.loginfo("Reading past boot interpreter prompt...")
        while l.startswith('>>>'):
            l = self.read_line()
            rospy.logdebug(l)

        rospy.loginfo('Sending Lego Hub main at {0}'.format(path))
        file_in = open(path, 'r')
        lines = file_in.readlines()
        file_in.close()
        rospy.loginfo("Entering paste mode...")
        self.write_byte(ctrl_e)
        l = self.read_line()
        while len(l.strip()) > 0:
            l = self.read_line()
            rate.sleep()
        for l in lines:
            # don't send empty lines
            check_l = l.rstrip()
            if len(check_l) > 0:
                rospy.logdebug(check_l)
                self.write_line(l)
                self.read_line()
                rate.sleep()
        self.read_line()

        rospy.loginfo("Exiting paste mode...")
        self.write_byte(ctrl_d)
        self.read_line()

        rospy.loginfo("Lego Hub main sent!")


    def run(self):
        "The main sensor-reading, command-sending loop.  The Lego Hub doesn't support threading, so this is a bit clunky"

        # our main sensor-reading loop runs at 10Hz
        # TODO: can we go faster?
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            datastr = self.read_line()
            try:
                data = eval(datastr)
                if len(data['err']) > 0:
                    for e in data['err']:
                        rospy.logerr('Error in response from hub: {0}'.format(e))

                self.send_ros_msgs(data)

                if len(self.command_queue) > 0:
                    self.command_queue.transmit(self)

            except Exception as err:
                rospy.logerr(err)
            rate.sleep()

        # cancel when we're done
        self.port.write(ctrl_c)

    def send_ros_msgs(self, data):
        "Publish the ROS messages"
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

        temperature_msg = Float32()
        temperature_msg.data = data['temperature']

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
            elif device['type'] == 'light':
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
        self.temperature_pub.publish(temperature_msg)
        self.joint_state_pub.publish(js)
        self.color_sensors_pub.publish(clr)
        self.distance_sensors_pub.publish(dst)

    def open(self):
        rospy.logerr("Not implented by this class")

    def close(self):
        rospy.logerr("Not implented by this class")

    def read_line(self):
        rospy.logerr("Not implented by this class")

    def write_line(self, txt):
        rospy.logerr("Not implented by this class")

    def write_byte(self, ch):
        rospy.logerr("Not implented by this class")

    def set_goal_positions(self, js):
        rospy.logwarn(js)
        param = {
            'name': list(js.name),
            'position': list(js.position),
            'velocity': list(js.velocity),
            'effort': list(js.effort)
        }
        rospy.logwarn(param)
        self.command_queue.append(CommandList.ACTION_MOTORS, param)

    def set_motor_config(self, cfg):
        param = {
            'name': cfg.name,
            'drive_mode': cfg.drive_mode,
            'stop_mode': cfg.stop_mode,
            'upper_limit': cfg.upper_limit,
            'lower_limit': cfg.lower_limit
        }
        self.command_queue.append(CommandList.ACTION_MOTOR_CONFIG, param)

    def set_lights(self, pattern):
        self.command_queue.append(CommandList.ACTION_LIGHTS, pattern.pattern)


class SerialInterface(LegoInterface):
    """Handles bidirectional communication over the USB virtual com port of the Lego Hub"""
    def __init__(self, port="/dev/lego", baud=115200, verbose=False):
        super().__init__()
        self.verbose = verbose

        self.port = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

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

    def read_line(self):
        'Reads a single line of text from the micropython interpreter'
        l = self.port.readline()
        l = l.decode('utf-8').rstrip()
        if self.verbose:
            rospy.loginfo(l)
        return l

    def write_line(self, txt):
        'Writes a single line of text with newline terminator to the micropython interpreter'
        data = bytes(txt, encoding='utf-8')
        self.port.write(data)

    def write_byte(self, ch):
        'Writes a single character to the micropython interpreter'
        self.port.write(ch)

    def send_main(self):
        path = catkin_find(project='lego_spike_interface', first_match_only=True, path='mindstorms/main.py')[0]
        super().send_main(path=path)
