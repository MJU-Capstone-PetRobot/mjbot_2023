#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/drcl/Desktop/mjbot_2023/
colcon build
source /home/drcl/Desktop/mjbot_2023/install/setup.bash
sleep 2
cd /home/drcl/Desktop/mjbot_2023/
ros2 run opi_esp opi_esp_comm