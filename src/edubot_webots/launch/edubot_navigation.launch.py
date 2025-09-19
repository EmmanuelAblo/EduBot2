from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    world_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/worlds/eduworld.wbt')
    nav2_params_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/config/nav2_params.yaml')
    rviz_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/rviz/edubot_rviz.rviz')
    urdf_path = os.path.expanduser('~/edubot_ws/src/edubot_webots/urdf/eduro_description.urdf')
    map_path = os.path.expanduser('~/edubot_ws/src/nav2/maps/map_final_webots02.yaml')

    return LaunchDescription([
        # Lancer Webots
        ExecuteProcess(
            cmd=['webots', world_path],
            output='screen'
        ),

        # TF joints et état robot
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path],
            parameters=[{'use_sim_time': True}]
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
        ),

        # Localisation Nav2 avec map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nav2_bringup"), '/launch', '/localization_launch.py'
            ]),
            launch_arguments={
                'params_file': nav2_params_path,
                'map': map_path,
                'use_sim_time': 'true'
            }.items()
        ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nav2_bringup"), '/launch', '/navigation_launch.py'
            ]),
            launch_arguments={
                'params_file': nav2_params_path,
                'use_sim_time': 'true'
            }.items()
        ),
        Node(
            package='edubot_webots',
            executable='initial_pose_publisher',
            name='initial_pose_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # RViz
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', rviz_path
            ],
            shell=True
        )
    ])
