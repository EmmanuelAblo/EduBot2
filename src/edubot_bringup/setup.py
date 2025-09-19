from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'edubot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    


    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')), 
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'edubot.rviz'), glob('edubot.rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Ghardallou',
    maintainer_email='mohamed.ghardallou@midlal.com',
    description='navy demo',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = edubot_bringup.my_node:main',
            'diff_driver = edubot_bringup.diff_driver:main',
            'joy_driver = edubot_bringup.joy_driver:main',
            'scan_filter = edubot_bringup.scan_filter:main',
            "initial_pose_publisher = edubot_webots.initial_pose_publisher:main"
        ],
    },
)
