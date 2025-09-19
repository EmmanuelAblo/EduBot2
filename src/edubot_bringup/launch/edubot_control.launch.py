#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from launch.actions import ExecuteProcess
def generate_launch_description():
    modbus_path = os.path.expanduser('~/edubot_ws/hardware/EduBot2_PID3')

    return LaunchDescription([

        # Drivers EduBot
        Node(
            package='edubot_bringup',
            executable='joy_driver',
            name='joy_driver',
        ),
         ExecuteProcess(
            cmd=[modbus_path],
            shell=True,
            output='screen'
        ),
        Node(
            package='edubot_bringup',
            executable='diff_driver',
            name='diff_driver',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='edubot_bringup',
            executable='scan_filter',
            name='scan_filter',
        )
    ])
