# #! /bin/bash
sleep 1
# source /opt/ros/humble/setup.bash

source /home/mju/mjbot_2023/install/setup.bash
sleep 1


cd /home/mju/mjbot_2023/
ros2 launch mjbot_bringup mjbot_bringup.launch.py
