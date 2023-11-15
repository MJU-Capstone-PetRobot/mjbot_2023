

  
## Environment

- ROS2: Humble Hawksbill
- OS:
    - Ubuntu 22.04 Jammy Jellyfish(Orange Pi Plus)

# Installation
### ROS2 Humble

I used this script

```shell
https://github.com/Tiryoh/ros2_setup_scripts_ubuntu
```

### download source code
```shell
git clone https://github.com/MJU-Capstone-PetRobot/mjbot_2023.git
```

### pip install
```shell
pip install -r requirements.txt
```

### run ./setup.sh

```shell
./setup.sh
```


# Install ROS 2 packages
sudo apt-get install ros-humble-dynamixel-sdk
sudo apt install ros-humble-dynamixel-workbench-toolbox
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-xacro



# Install additional ROS 2 packages and dependencies
sudo apt install ros-humble-joy-linux
sudo apt install ros-humble-twist-mux
sudo -H apt-get install -y ros-humble-joint-state-publisher
sudo -H apt-get install -y python3-serial

sudo apt-get install ffmpeg
sudo apt-get install libasound-dev
sudo apt-get install portaudio19-dev
