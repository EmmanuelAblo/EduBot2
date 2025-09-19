#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    world_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/worlds/eduworld.wbt')
    urdf_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/urdf/eduro_description.urdf')
    mapper_params_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/config/mapper_params_online_async.yaml')
    rviz_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/rviz/edubot_rviz.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='slam_params',
            default_value=mapper_params_path,
            description='Fichier de paramètres pour SLAM Toolbox'
        ),

        ExecuteProcess(
            cmd=['webots', world_path],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path],
        ),

        Node(
            package='edubot_webots',
            executable='cmd_vel_listener',
            name='cmd_vel_listener',
        ),

        Node(
            package='edubot_webots',
            executable='joy_driver',
            name='joy_driver',
            output='screen'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Lancement direct de async_slam_toolbox_node avec le fichier YAML custom
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[mapper_params_path],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', rviz_path
            ],
            shell=True
        )
    ])
