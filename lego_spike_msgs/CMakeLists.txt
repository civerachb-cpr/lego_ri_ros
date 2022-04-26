cmake_minimum_required(VERSION 3.5)
project(lego_spike_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  Color.msg
  ColorSensors.msg
  DistanceSensors.msg
  MotorCfg.msg
)

generate_messages(
  DEPENDENCIES std_msgs)

catkin_package()
