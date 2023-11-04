#! /bin/bash
source /opt/ros/humble/setup.bash

cd /home/mju/Desktop/mjbot_2023/
colcon build

source /home/mju/Desktop/mjbot_2023/install/setup.bash

sleep 2

cd /home/mju/Desktop/mjbot_2023/
ros2 launch mjbot_bringup mjbot_bringup.launch.py
