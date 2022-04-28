cmake_minimum_required(VERSION 3.0.2)
project(lego_spike_interface)

cmake_policy(SET CMP0057 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(lego_spike_msgs REQUIRED)

install(DIRECTORY
  mindstorms
  DESTINATION share/${PROJECT_NAME}
)

ament_package()