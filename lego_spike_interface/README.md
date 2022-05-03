Lego Spike Interface
======================

This package contains both the ROS node that interacts with the Lego Mindstorms or Lego Spike Prime hub over a USB
connection and the MicroPython code that runs on the hub itself.

Usage
--------

Connect the Mindstorms Hub to your PC using the micro USB cable.  Start the driver with
```
roslaunch lego_spike_interface serial_interface.launch
```
or
```
rosrun lego_spike_interface serial_interface
```

By default the driver opens `/dev/lego` (symlink provided by the udev rule found in `lego_spike_interface/debian`) but
this can be configured using the `port` argument.  Baud rate is configurable for custom firmware, but currently
only `115200` shoud be used.


ROS Topics
------------

The `SerialInterface` class reads the raw data as described above and publishes the data on the following topics:
- `/colors` -- Color & light-level data from all connected light sensors
- `/distance` -- Distance reported by all ultrasonic distance sensors
- `/imu/data` -- Internal IMU gyroscope and accelerometer data
- `/joint_states` -- Joint positions and velocities of all connected motors

The following ROS topics are used for input:
- `/cmd/goal_position` -- move the joints to the specified positions or at the specified speeds
- `/cmd/lights` -- send a 25-item row-major array to be displayed on the LED grid. Values should be brightness levels 0-9

The Lego motors provide both servo-like position control, operating from -180 to 180 degrees, or continuous, wheel-like
drive.  Both modes are suported by the `cmd/goal_position` topic:

- to move the motor to a specific angle, set the position variable to the desired angle, the effort to 1.0, and speed to 0.0
- to move the motor continuously at a specific speed, set set the speed to the desired speed, and the effort to 1.0. The position value is ignored
- to float the motor set the effort to 0.0. The position and speed will be ignored
- to lock the motor, set the effort to -1.0

Note that the speed is expressed as a [-1, 1] value indicating the percentage of maximum design speed; the actual maximum
speed appears to be undocumented, but if I ever figure it out I'll change it to use real units. Overall this implementation
is a bit clunky, and will likely be improved later on. But it's usable at least, if a little unconventional.

Motor joint names begin with `motor_{a|b|c|d|e|f}` according to the port they are
attached to.  Below are several examples of controlling motor behaviour:

Set a motor to the zero position:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "name: ['motor_a_wheel_joint']
position: [0]
velocity: [0]
effort: [1]" -1
```

Turn a motor to 90 degrees:
Set a motor to the zero position:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "name: ['motor_a_wheel_joint']
position: [$(deg2rad 90)]
velocity: [0]
effort: [1]" -1
```

Rotate a motor continually at 20% of top speed:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "name: ['motor_a_wheel_joint']
position: [0]
velocity: [0.2]
effort: [1]" -1
```

Float the motor (parks it, but you can still turn it by hand)
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "name: ['motor_a_wheel_joint']
position: [0]
velocity: [0]
effort: [0]" -1
```

Actively hold the motor at its current position:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "name: ['motor_a_wheel_joint']
position: [0]
velocity: [0]
effort: [-1]" -1
```

Example of setting 4 motors to different angles:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "name: ['motor_a_wheel_joint', 'motor_b_wheel_joint', 'motor_c_wheel_joint', motor_d_wheel_joint]
position: [$(deg2rad 90), $(deg2rad 45), $(deg2rad 0), $(deg2rad -45)]
velocity: [0, 0, 0, 0]
effort: [1, 1, 1, 1]" -1
```

Setting motors A and B to move to 90, motor C to spin one direction, and motor D the other:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "name: ['motor_a_wheel_joint', 'motor_b_wheel_joint', 'motor_c_wheel_joint', motor_d_wheel_joint]
position: [$(deg2rad 90), $(deg2rad 90), 0, 0]
velocity: [0, 0, 0.3, -0.1]
effort: [1, 1, 1, 1]" -1
```

Hold motor A, float motor B, spin motor C, move motor D to -45 degrees:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "name: ['motor_a_wheel_joint', 'motor_b_wheel_joint', 'motor_c_wheel_joint', motor_d_wheel_joint]
position: [0, 0, 0, $(deg2rad -45)]
velocity: [0, 0, 0.5, 0]
effort: [-1, 0, 1, 1]" -1
```


Data Flow
-----------

Data between the hub and ROS PC is encoded as UTF-8 strings, with each packet terminating with a newline character.

The hub uses a single-threaded main-loop style of program (the MicroPython implementation does not appear to support
threading at this time).  It attempts to read a line of text from the serial interface, decodes if if necessary,
invokes the necessary actions to control the motors/lights, and finally sends back a text-encoded `dict` object
containing the status of all of the motors and sensors. This `dict` object can be `eval`'d on the PC and parsed
as needed.  The `dict` contains these fields:
- `imu` -- IMU data from the hub encoded in a format smilar to a ROS `sensor_msgs/Imu` message:
  - `linear` containing `x`, `y` and `z` linear accelerations in m/s^2
  - `angular` containing `x`, `y` and `x` angular velocities in deg/s
- `temperature` -- the temperature of the hub in C
- `devices` -- an array of up to 6 items each representing a Lego Spike Prime or Lego Mindstorms compatible sensor/motor
- `err` -- an array of error messages from the hub for debugging/diagnostics

The `devices` object contains these fields:
- `type` -- indicates the type of device. Currently one of `motor`, `distance`, and `light` are supported
- `port` -- the port `a`-`f` that the device is connected to
- `data` -- the raw data from the device.  See below for `type`-specific details

The `data` field will contain the following based on `type`:
- `motor`
  - `position` -- the current angle of the motor in degrees from -180 to 180
  - `speed` -- a value from -100 to 100 indicating the motor's speed as a percentage of design tolerance
- `distance`
  - the distance measured in meters, or `NaN` if no reading was able to be made (e.g. out of range)
- `light`
  - `level` -- a level from 0 to 100 indicating the overall brightness
  - `rgb` -- a `dict` containing `r`, `g`, and `b` fields from 0-255 indicating the RGB channel data

Note that degrees are used to reduce the potential length of the encoded data.  Also the Lego motors are not that
precise and may not be reliable at angles smaller than 1 degree increments.

Data sent from the PC to the Hub uses a similar format of a `dict` encoded as a UTF-8 string:
- `actions` -- the array of command types as strings. Valid types are `lights` and `motors`
- `parameters` -- an array of parameters associated with the actions:
  - `motors` -- a `dict` containing the following arrays:
    - `name` -- names of the motors to set, e.g. `motor_a`, `motor_b`, etc...
    - `position` -- the desired position of the motors in degrees
    - `velocity` -- the desired velocity as a -100 to 100 percentage of maximum design speed. Set to 0 for position
      control
    - `effort` -- 0 to float the motor, 1 to use speed/position control, -1 to actively hold current position
  - `lights` -- an array of integers of length 25 indicating the desired light pattern. Values should be in the range 0
    (off) to 9 (full brightness)
