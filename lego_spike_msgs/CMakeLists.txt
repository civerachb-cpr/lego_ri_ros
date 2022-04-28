cmake_minimum_required(VERSION 3.5)
project(lego_spike_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Color.msg"
  "msg/ColorSensors.msg"
  "msg/DistanceSensors.msg"
  "msg/LightPattern.msg"
)

ament_package()
