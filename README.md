## Document
[User Guide](User_Guide.md)

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

## Mjbot Workspace

### Create dir & download source code
```shell
mkdir ~/mjbot_ws && cd ~/mjbot_ws
mkdir src && cd src
git clone https://github.com/MJU-Capstone-PetRobot/mjbot_2023.git
```


### install dependencies

```shell
cd ~/mjbot_ws
./src/mjbot_2023/setup.sh
pip install -r ./src/mjbot_2023/requirements.txt
```

### build
```shell
colcon build --symlink-install
```

### sourcing

```shell
source ~/mjbot_ws/install/setup.bash
```


## Launch

```shell
ros2 launch mjbot_bringup mjbot_bringup.launch.py
```

## .ENV for API KEY

### Install 

```python
pip3 install python-dotenv
```

### Create
```shell
cd ~/mjbot_ws
vim .ENV
```

### Write

```shell
GPT_API = "API KEY"
CLIENT_ID = "API KEY"
CLIENT_SECRET = "API KEY"
```
