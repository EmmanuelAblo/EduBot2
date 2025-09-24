# 🤖 EduBot2 – Autonomous Mobile Robot

EduBot2 is a differential-drive autonomous robot developed as part of a Master’s thesis project.  
It includes both **simulation in Webots** and **real-world deployment with ROS 2 (Humble)**.  
The system integrates **SLAM, autonomous navigation, joystick teleoperation, and motor control via Modbus TCP**.  

---

## 📦 Packages Overview  

### 🔹 edubot_webots (Simulation in Webots + ROS 2)
This package allows EduBot2 to be simulated in Webots with ROS 2 integration.

Main Node:

cmd_vel_listener.py

Controls EduBot2 in Webots by subscribing to /cmd_vel

Publishes: /odom, /scan, /clock, and TFs

Odometry is computed using Webots wheel encoders

Launch files:

```bash
ros2 launch edubot_webots edubot_slam.launch.py        # SLAM (map building)
ros2 launch edubot_webots edubot_navigation.launch.py  # Navigation with pre-existing map
ros2 launch edubot_webots edubot_control.launch.py     # Control with PS4 joystick
ros2 launch edubot_webots edubotkey_launch.py          # Control with keyboard
ros2 launch edubot_webots edubotps3joy_launch.py       # Control with PS3 joystick
```
### 🔹 rplidar_ros (LiDAR Driver)
Used to operate the RPLidar A2M8 sensor.

Launch command:

```bash
ros2 launch rplidar_ros rplidar_a2m8_launch.py inverted:=True
```
Documentation: .[ROS 2 RPLidar Docs](https://docs.ros.org/en/humble/p/rplidar_ros/index.html)

🗺️ Map Saving (Real Robot)
After exploring with SLAM Toolbox, you can save the map using:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/edubot_ws/src/nav2/maps/map_name
```
# 🧪 MATLAB + Webots Controllers
Located in the folder: edubot2_webot

Includes the Webots world file, EduBot2 prototype, and 4 MATLAB controllers:

controlleur_AMCL → Adaptive Monte Carlo Localization

docking0_controller → Automatic docking

my_controller0 → Basic robot control + logging odometry & SLAM data for offline use in SLAM Builder App

odom_realtime → Real-time odometry visualization (compared to real robot in webots)

realtime_SLAM → Real-time SLAM execution

### Run controller from terminal (Windows example):

```bash
webots-controller.exe path/to/controller/file
```
Example:
```bash
webots-controller.exe C:\...\edubot2_webot\controllers\controlleur_AMCL\controlleur_AMCL.m
```
Run in interactive mode:
```bash
webots-controller.exe --interactive C:\...\edubot2_webot\controllers\controlleur_AMCL\controlleur_AMCL.m
```

## 🚀 Features
✅ ROS 2 Integration (simulated robot)

✅ Webots simulation with SLAM Toolbox & Nav2

✅ Joystick & keyboard teleoperation

✅ Differential drive control with Modbus TCP

✅ LiDAR integration & filtering

✅ Map saving & navigation with pre-built maps

✅ MATLAB controllers for AMCL, SLAM, and docking

## 📂 Project Structure
```bash
edubot_ws/
│── src/
│   ├── edubot_webots/       # Webots simulation nodes & launch files
│   ├── rplidar_ros/         # RPLidar driver package
│
├── edubot2_webot/           # MATLAB + Webots world + controllers
│   ├── controllers/
│   │   ├── controlleur_AMCL/
│   │   ├── docking0_controller/
│   │   ├── my_controller0/
│   │   ├── odom_realtime/
│   │   ├── realtime_SLAM/
```
## 📜 License
This project is for academic and research purposes.
Feel free to use and adapt it with proper citation.
