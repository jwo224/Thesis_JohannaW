import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'gr4_trolley_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group4',
    maintainer_email='group4@example.com',
    description='Real robot trolley ArUco vision launch and test utilities.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_camera_node = gr4_trolley_vision.opencv_camera_node:main',
            'aruco_detection_monitor = gr4_trolley_vision.aruco_detection_monitor:main',
        ],
    },
)
