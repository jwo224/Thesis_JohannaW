from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'gr4_camera_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocket',
    maintainer_email='heimir0509@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
           'identify_cameras = gr4_camera_config.identify_cameras:main',
           'front_rear_camera_node = gr4_camera_config.front_rear_camera_node:main',
           'camera_web_stream = gr4_camera_config.camera_web_stream:main',
           'aruco_web_stream = gr4_camera_config.camera_web_stream:main',
           'trolley_reference_logger = gr4_camera_config.trolley_reference_logger:main',
           'physical_four_camera_node = gr4_camera_config.physical_four_camera_node:main',
           'single_aruco_reference_logger = gr4_camera_config.single_aruco_reference_logger:main',
           'physical_trolley_alignment_controller = gr4_camera_config.physical_trolley_alignment_controller:main',
         ],
    },
)

