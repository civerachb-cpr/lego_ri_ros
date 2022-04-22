cmake_minimum_required(VERSION 3.0.2)
project(lego_spike_interface)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
)

include_directories(
)

install(FILES
  scripts/serial_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
