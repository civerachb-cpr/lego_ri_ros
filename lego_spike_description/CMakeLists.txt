cmake_minimum_required(VERSION 3.0.2)
project(lego_spike_description)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

install(DIRECTORY urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
