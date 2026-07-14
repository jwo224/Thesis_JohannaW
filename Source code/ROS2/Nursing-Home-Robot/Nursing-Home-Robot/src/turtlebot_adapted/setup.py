import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'turtlebot_adapted'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group4',
    maintainer_email='johanna.woerz07@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'zones_and_delivery = turtlebot_adapted.zones_and_delivery:main',
        'spawn_zone_marks = turtlebot_adapted.spawn_zone_marks:main',
        'robot_shell_spawner = turtlebot_adapted.robot_shell_spawner:main',
        ],
    },
)
