cmake_minimum_required(VERSION 3.5)
project(mjbot_alert)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    Alert
  DESTINATION
    lib/${PROJECT_NAME}/
)

install(
  PROGRAMS
    mjbot_alert.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
