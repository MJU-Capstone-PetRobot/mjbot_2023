# mjbot_description

## Description
This package holds the urdf description of the robot.

<img src="docs/robot_rviz.png">

## Launch Files

For launching robot state publisher for filling up static tf information and serving the description of the robot. Typically used during robot bringup.
```
ros2 launch mjbot_description mjbot_description.launch.py
```

For launching the robot state publisher and providing some visualization with rviz to analyze the robot description.
```
ros2 launch mjbot_description view_mjbot.launch.py
```


