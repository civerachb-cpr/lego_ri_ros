Lego Robot Inventor ROS
=========================

This repo contains code needed for operating the Lego Robot Inventor (#51515)
with ROS.

Everything is very much in development right now, so don't expect much to
work yet.

The intention is that the Mindstorms Hub will operate as a serial device,
sending and receiving messages over the USB (eventually maybe wi-fi and
bluetooth too), with the actual ROS data processing being done on the main
PC.  No ROS-specific code runs on the Mindstorms Hub.

Note that the Lego Spike Prime and Lego Mindstorms hardware is interchangeable.

Only one hub per PC is currently supported, though that may change in the future.


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

Topics
--------

Sensor data from the Hub is published on a variety of topics:

- `/colors` -- Color & light-level data from all connected light sensors
- `/distance` -- Distance reported by all ultrasonic distance sensors
- `/imu/data` -- Internal IMU gyroscope and accelerometer data
- `/joint_states` -- Joint positions and velocities of all connected motors

Topics under the `cmd` namespace are used for sending instructions to the Hub:
- `/cmd/goal_position` -- move the joints to the specified positions
- `/cmd/lights` -- send a 25-item row-major array to be displayed on the LED grid. Values should be brightness levels 0-9
- `/cmd/motor_config` -- not currently implemented, but will eventually be used to change between continuous and angle-limited motor configurations

Currently only the position value of `cmd/goal_position` is used; effort and velocity must be set, but are ignored by
the underlying message-handler.  Motor names should begin with `motor_{a|b|c|d|e|f}` according to the port they are
attached to

Example of setting 4 motors to zero:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['motor_a_wheel_link', 'motor_b_wheel_link', 'motor_c_wheel_link', 'motor_d_wheel_link']
position: [0, 0, 0, 0]
velocity: [0, 0, 0, 0]
effort: [0, 0, 0, 0]" -1
```

Example of setting each motor to a different position:
```
$ rostopic pub /cmd/goal_position sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['motor_a_wheel_link', 'motor_b_wheel_link', 'motor_c_wheel_link', 'motor_d_wheel_link']
position: [$(deg2rad 45), $(deg2rad 90), $(deg2rad -90), $(deg2rad -45)]
velocity: [0, 0, 0, 0]
effort: [0, 0, 0, 0]" -1
```

Sources
---------

The following links contain useful information for working with the Lego Mindstorms Hub:

- Lego Mindstorms Robot Inventor https://www.lego.com/en-us/product/robot-inventor-51515
- Lego Spike Prime https://education.lego.com/en-us/products/lego-education-spike-prime-set/45678#spike%E2%84%A2-prime
- Lego Mindstorms Hub API https://lego.github.io/MINDSTORMS-Robot-Inventor-hub-API
- Lego Hub technical specs https://le-www-live-s.legocdn.com/sc/media/files/support/spike-prime/techspecs_techniclargehub-fba3b469ecb9eaafbde5f24d34ba090e.pdf
- Robot Inventor Tools https://github.com/ckumpe/robot-inventor-tools
