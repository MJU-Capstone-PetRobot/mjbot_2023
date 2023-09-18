cmake_minimum_required(VERSION 3.8)
project(mjbot_neck_control)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

install(
  DIRECTORY
    src
  DESTINATION
    lib/${PROJECT_NAME}/
)


install(
  PROGRAMS
  src/Neck_control.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
