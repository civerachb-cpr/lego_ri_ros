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


Sources
---------

The following links contain useful information for working with the Lego Mindstorms Hub:

- Lego Mindstorms Robot Inventor https://www.lego.com/en-us/product/robot-inventor-51515
- Lego Spike Prime https://education.lego.com/en-us/products/lego-education-spike-prime-set/45678#spike%E2%84%A2-prime
- Lego Mindstorms Hub API https://lego.github.io/MINDSTORMS-Robot-Inventor-hub-API
- Lego Hub technical specs https://le-www-live-s.legocdn.com/sc/media/files/support/spike-prime/techspecs_techniclargehub-fba3b469ecb9eaafbde5f24d34ba090e.pdf
- Robot Inventor Tools https://github.com/ckumpe/robot-inventor-tools
