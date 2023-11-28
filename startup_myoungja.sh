#! /bin/bash
sleep 2
source /opt/ros/humble/setup.bash

source /home/mju/mjbot_2023/install/setup.bash

sleep 2

cd /home/mju/mjbot_2023/
ros2 launch mjbot_bringup mjbot_bringup.launch.py
