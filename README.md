## Document
[User Guide](User_Guide.md)  
[User GUIDE English.ver](User_Guide_en.md)

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
git clone https://github.com/MJU-Capstone-PetRobot/mjbot_2023.git
```


### install dependencies

```shell
./setup.sh
pip install -r ./requirements.txt
```

### build
```shell
colcon build --symlink-install
```

### sourcing

```shell
source ./install/setup.bash
```


## Launch

```shell
ros2 launch mjbot_bringup mjbot_bringup.launch.py
```

## .env for API KEY

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
# 위험 알림 API KEY
GOOGLE_API = "API KEY"
TWILIO_SID = "API KEY"
TWILIO_TOKEN = "API KEY"
FROM_TWILIO = "API KEY"
TO_TWILIO = "API KEY"
# 대화 API KEY
GPT_API = "API KEY"
CLIENT_ID = "API KEY"
CLIENT_SECRET = "API KEY"
```
