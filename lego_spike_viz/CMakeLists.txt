cmake_minimum_required(VERSION 3.0.2)
project(lego_spike_viz)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

install(DIRECTORY config meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
