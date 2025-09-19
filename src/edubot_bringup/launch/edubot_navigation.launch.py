from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    nav2_params_path = os.path.expanduser('~/edubot_ws/src/edubot_bringup/config/nav2_params.yaml')
    rviz_path = os.path.expanduser('~/edubot_ws/src/edubot_bringup/edubot.rviz/edubot_rviz.rviz')
    map_path = os.path.expanduser('~/edubot_ws/src/nav2/maps/map_final_lab.yaml')
    urdf_path = os.path.expanduser('~/edubot_ws/src/edubot_bringup/urdf/eduro_description.urdf')
    modbus_path = os.path.expanduser('~/edubot_ws/hardware/EduBot2_PID3')

 
    return LaunchDescription([
        # Lancer Modbus (EduBot2_PID3)
        ExecuteProcess(
            cmd=[modbus_path],
            shell=True,
            output='screen'
        ),

        # Lancer RPLiDAR avec inversion
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'rplidar_ros', 'rplidar_a2m8_launch.py',
                'inverted:=true'
            ],
            shell=True,
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_link',
            arguments=['0.1395', '0', '0.187', '0', '-3.1415', '0', 'base_link', 'lidar_link'],
        ),

        # Drivers EduBot
        Node(
            package='edubot_bringup',
            executable='joy_driver',
            name='joy_driver',
        ),
        Node(
            package='edubot_bringup',
            executable='diff_driver',
            name='diff_driver',
        ),
        Node(
            package='edubot_bringup',
            executable='scan_filter',
            name='scan_filter',
        ),  
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("nav2_bringup"), '/launch', '/localization_launch.py']
            ),
            launch_arguments={
                'params_file': nav2_params_path,
                'map': map_path
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("nav2_bringup"), '/launch', '/navigation_launch.py']
            ),
            launch_arguments={
                'params_file': nav2_params_path
            }.items()
        ),
        #publier la position initial a 0,0,0
        Node(
            package='edubot_webots',
            executable='initial_pose_publisher',
            name='initial_pose_publisher',
            output='screen',
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