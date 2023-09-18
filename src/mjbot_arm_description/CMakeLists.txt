cmake_minimum_required(VERSION 3.5)
project(mjbot_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config rviz gazebo ros2_control urdf worlds map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
