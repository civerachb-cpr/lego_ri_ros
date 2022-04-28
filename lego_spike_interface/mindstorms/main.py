#!/usr/bin/env mpython
"""
Simple main-loop program that runs on the Lego Mindstorms Hub and
prints status messages at approx. 10Hz.

Once per loop it accepts input from STDIN for processing
"""

import hub

from math import pi

running_pattern = [
    9,0,7,0,5,
    0,0,8,0,6,
    7,8,9,0,7,
    0,0,0,0,8,
    5,6,7,8,9
]

def set_led_pattern(pattern):
    for px in range(len(pattern)):
        x = int(px % 5)
        y = int(px / 5)
        gamma = pattern[px]
        hub.display.pixel(x, y, gamma)

class Motor:
    def __init__(self, device, data, port):
        self.__device = device
        self.__type = 'motor'
        self.__port = port

    def get(self):
        speed_percent, net_encoder_pos, angle_degrees, speed_deg_per_s = self.__device.get()

        angle_rad = angle_degrees * pi / 180.0
        speed_rad_per_s = speed_deg_per_s * pi / 180

        return {
            'position': angle_rad,
            'speed'   : speed_rad_per_s
        }


class LightSensor:
    def __init__(self, device, data, port):
        self.__device = device
        self.__type='light'
        self.__port = port

    def get(self):
        brightness, idk_wtf_this_is_yet, r, g, b = self.__device.get()
        return {
            'level': brightness,
            'rgb': {
                'r': r / 1024.0,
                'g': g / 1024.0,
                'b': b / 1024.0
            }
        }


class DistanceSensor        :
    def __init__(self, device, data, port):
        self.__device = device
        self.__type='distance'
        self.__port = port

    def get(self):
        raw_data = self.__device.get()
        if raw_data[0] == None:
            return -1.0
        else:
            return raw_data[0] * 0.01  # base device returns cm, convert to m

def enumerate_devices():
    ports = {
        'A': hub.port.A.device,
        'B': hub.port.B.device,
        'C': hub.port.C.device,
        'D': hub.port.D.device,
        'E': hub.port.E.device,
        'F': hub.port.F.device
    }

    devices = {
        'A': None,
        'B': None,
        'C': None,
        'D': None,
        'E': None,
        'F': None
    }

    for p in ports.keys():
        if ports[p] is not None:
            data = ports[p].get()
            if len(data) == 4:
                devices[p] = Motor(ports[p], data, p.lower())
            elif len(data) == 1:
                devices[p] = DistanceSensor(ports[p], data, p.lower())
            elif len(data) == 5:
                devices[p] = LightSensor(ports[p], data, p.lower())

    return devices

def read_devices():
    data = []
    for d in devices.keys():
        dev = devices[d]
        if dev is None:
            data.append(None)
        else:
            data.append({
                'type': dev.__type,
                'data': dev.get(),
                'port': dev.__port
            })
    return data

def read_gyro():
    status = hub.status()
    return {
        'angular': {
            'x': status['gyroscope'][0] * pi / 180.0,
            'y': status['gyroscope'][1] * pi / 180.0,
            'z': status['gyroscope'][2] * pi / 180.0
        },
        'linear': {
            # reported in cm/s^2
            'x': status['accelerometer'][0] / 100.0,
            'y': status['accelerometer'][1] / 100.0,
            'z': status['accelerometer'][2] / 100.0
        }
    }

def read_temperature():
    return hub.temperature()

def read_cmd():
    data = port.readline()
    if data is not None:
        data = data.decode('utf-8')
    else:
        data = ''
    return data

def run_cmds(cmdstr):
    errs = []
    try:
        cmds = eval(cmdstr)
        if len(cmds['actions']) == len(cmds['parameters']):
            for i in range(len(cmds['actions'])):
                action = cmds['actions'][i]
                param = cmds['parameters'][i]

                errs.append('action: {0}, param: {1}'.format(action, param))
                if action == 'lights':
                    set_led_pattern(param)
        else:
            errs.append('Action/Parameter length mismatch')
    except Exception as e:
        errs.append(str(e))
    return errs

################################################################################
## MAIN
################################################################################

port = hub.USB_VCP(0)
devices = enumerate_devices()

set_led_pattern(running_pattern)
while True:
    cmd = read_cmd()
    errs = []
    if len(cmd) > 0:
        errs = run_cmds(cmd)

    data = {
        'imu': read_gyro(),
        'temperature': read_temperature(),
        'devices': read_devices(),
        'err': errs
    }

    print(data)
