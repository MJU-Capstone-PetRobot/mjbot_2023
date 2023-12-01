#! /bin/bash
sleep 1
source /opt/ros/humble/setup.bash
sleep 1

source /home/mju/mjbot_2023/install/setup.bash
sleep 3


cd /home/mju/mjbot_2023/
ros2 launch mjbot_bringup mjbot_bringup.launch.py
