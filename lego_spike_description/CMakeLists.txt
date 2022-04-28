cmake_minimum_required(VERSION 3.0.2)
project(lego_spike_description)

cmake_policy(SET CMP0057 NEW)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package(
)