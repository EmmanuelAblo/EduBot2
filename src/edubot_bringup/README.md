# edubot_bringup

ROS 2 bringup package for the EduBot2 real robot.  
Includes drivers, teleop, and LiDAR filtering.

## Features
- Modbus TCP motor driver (`diff_driver.py`)
- Joystick teleoperation (`joy_driver.py`)
- LiDAR filtering node (`scan_filter.py`)
- Launch files for SLAM, Navigation, and Control

## Run examples
```bash
ros2 launch edubot_bringup edubot_control.launch.py
ros2 launch edubot_bringup edubot_navigation.launch.py
ros2 launch edubot_bringup edubot_slam.launch.py
```
