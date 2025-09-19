#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/worlds/eduworld.wbt')
    urdf_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/urdf/eduro_description.urdf')
    mapper_params_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/config/mapper_params_online_async.yaml')
    nav2_params_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/config/nav2_params.yaml')
    rviz_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/rviz/edubot_rviz.rviz')

    return LaunchDescription([
        # Lancement de Webots
        ExecuteProcess(
            cmd=['webots', world_path],
            output='screen'
        ),
        #Joint state publisher (publie l'état des joints du robot à partir du URDF)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        # Robot state publisher (TF à partir du URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path],
        ),
        # Ton nœud pour écouter /cmd_vel
        Node(
            package='edubot_webots',
            executable='cmd_vel_listener',
            name='cmd_vel_listener',
        ),

        # Joystick
        Node(
            package='edubot_webots',
            executable='edubot_teleopjoy',
            name='edubot_teleopjoy',
            output='screen'
        ),
        # SLAM Toolbox (cartographie en ligne)
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                f'params_file:={mapper_params_path}',
                'use_sim_time:=True'
            ],
            shell=True
        ),

        # Nav2 (navigation)
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
                f'params_file:={nav2_params_path}',
                'use_sim_time:=True'
            ],
            shell=True
        ),

        # RViz (visualisation)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', rviz_path
            ],
            shell=True
        )
    ])
