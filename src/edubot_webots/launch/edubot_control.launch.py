#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/worlds/eduworld.wbt')
    urdf_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/urdf/eduro_description.urdf')
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
        # Drivers EduBot
        Node(
            package='edubot_webots',
            executable='joy_driver',
            name='joy_driver',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='edubot_webots',
            executable='cmd_vel_listener',
            name='cmd_vel_listener',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])
