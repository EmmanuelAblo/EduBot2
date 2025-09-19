from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    urdf_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/urdf/eduro_description.urdf')
    mapper_params_path = os.path.expanduser('~/edubot_ws/src/edubot_bringup/config/mapper_params_online_async.yaml')
    nav2_params_path = os.path.expanduser('~/edubot_ws/src/edubot_bringup/config/nav2_params.yaml')
    rviz_path = os.path.expanduser('~/edubot_ws/src/edubot_bringup/edubot.rviz/edubot_rviz.rviz')
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

        # TF statiques
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

        # SLAM Toolbox (cartographie en ligne)
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                f'params_file:={mapper_params_path}',
                'use_sim_time:=false'
            ],
            shell=True
        ),

        # # Nav2 (navigation)
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
        #         f'params_file:={nav2_params_path}',
        #     ],
        #     shell=True
        # ),

        # RViz (visualisation)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', rviz_path
            ],
            shell=True
        )
    ])
