#!/bin/bash
set -e

sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

# Install initial dependencies
sudo apt-get install -y python3 python3-venv python3-pyaudio libportaudio2 libportaudiocpp0 portaudio19-dev libatlas-base-dev ffmpeg flac sense-hat

# Install additional ROS 2 packages for Dynamixel, control, and xacro
sudo apt-get install ros-humble-dynamixel-sdk
sudo apt install ros-humble-dynamixel-workbench-toolbox
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-xacro

# Install more ROS 2 packages and dependencies
sudo apt install ros-humble-joy-linux
sudo apt install ros-humble-twist-mux
sudo -H apt-get install -y ros-humble-joint-state-publisher
sudo -H apt-get install -y python3-serial
sudo apt-get install python3-pip
sudo apt install -y cmake
sudo apt install ros-humble-hardware-interface
sudo apt-get install libportaudio2
sudo apt-get install libasound-dev
