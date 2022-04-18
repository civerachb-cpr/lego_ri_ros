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
