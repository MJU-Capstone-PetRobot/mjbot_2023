# Installation

- ROS2: Humble Hawksbill
- OS:
    - Ubuntu 22.04 Jammy Jellyfish(Orange Pi Plus)


## Environment

### ROS2 Humble

https://docs.ros.org/en/humble/Installation.html


# TODO

Team Commit message Rule?


Don't upload build files


# Install ROS 2 packages
sudo apt-get install ros-humble-dynamixel-sdk
sudo apt install ros-humble-dynamixel-workbench-toolbox
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-xacro

# Install setuptools version 58.2.0 using pip
pip install setuptools==58.2.0

# Make the USB device accessible
sudo chmod a+rw /dev/ttyUSB0


# Install additional ROS 2 packages and dependencies
sudo apt install ros-humble-joy-linux
sudo apt install ros-humble-twist-mux
sudo -H apt-get install -y ros-humble-joint-state-publisher
sudo -H apt-get install -y python3-serial

sudo apt-get install ffmpeg
sudo apt-get install libasound-dev
sudo apt-get install portaudio19-dev