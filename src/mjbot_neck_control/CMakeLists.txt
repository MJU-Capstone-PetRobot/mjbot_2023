cmake_minimum_required(VERSION 3.8)
project(mjbot_neck_control)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)


install(
  PROGRAMS
  mjbot_neck_control/person_tracking_launcher.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
