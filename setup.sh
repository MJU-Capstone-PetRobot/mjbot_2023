#!/bin/bash
set -e

# Update package lists
sudo apt-get update
rosdep update

# Install dependencies for ROS and other required packages
sudo apt-get install -y \
    gstreamer-1.0 \
    python3 \
    python3-venv \
    python3-pyaudio \
    libportaudio2 \
    libportaudiocpp0 \
    portaudio19-dev \
    libatlas-base-dev \
    ffmpeg \
    flac \
    sense-hat \
    python3-dev \
    python3-pip \
    gcc \
    python3-opencv \
    python3-numpy \
    python3-setuptools \
    python3-dev \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    cmake \
    libasound-dev \
    python3-serial

# Install ROS 2 packages
sudo apt-get install -y \
    ros-humble-dynamixel-sdk \
    ros-humble-dynamixel-workbench-toolbox \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joy-linux \
    ros-humble-twist-mux \
    ros-humble-joint-state-publisher \
    ros-humble-hardware-interface

# Install dependencies from rosdep
rosdep install --from-paths src --ignore-src -y
