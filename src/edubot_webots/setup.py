from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'edubot_webots'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Fichiers ROS2 standard
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),     
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'protos'), glob('protos/*.proto')),
        (os.path.join('share', package_name, 'edubot.rviz'), glob('edubot.rviz/*')),
        # Meshes STL
        (os.path.join('share', package_name, 'protos/meshes'), glob('protos/meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amvdeus',
    maintainer_email='amvdeus@todo.todo',
    description='ROS 2 interface for EduBot robot in Webots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "edubot_teleopjoy = edubot_webots.edubot_teleopjoy:main",
            "cmd_vel_listener = edubot_webots.cmd_vel_listener:main",
            "edubot_teleopkey = edubot_webots.edubot_teleopkey:main",
            "joy_driver = edubot_webots.joy_driver:main",
            "initial_pose_publisher = edubot_webots.initial_pose_publisher:main"
        ],
    },
)
