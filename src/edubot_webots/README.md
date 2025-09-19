# 🔹 edubot_webots (Simulation in Webots + ROS 2)
This package allows EduBot2 to be simulated in Webots with ROS 2 integration.

## Main Node:

cmd_vel_listener.py

Controls EduBot2 in Webots by subscribing to /cmd_vel

Publishes: /odom, /scan, /clock, and TFs

Odometry is computed using Webots wheel encoders

### Launch files:

```bash
ros2 launch edubot_webots edubot_slam.launch.py        # SLAM (map building)
ros2 launch edubot_webots edubot_navigation.launch.py  # Navigation with pre-existing map
ros2 launch edubot_webots edubot_control.launch.py     # Control with PS4 joystick
ros2 launch edubot_webots edubotkey_launch.py          # Control with keyboard
ros2 launch edubot_webots edubotps3joy_launch.py       # Control with PS3 joystick
```
