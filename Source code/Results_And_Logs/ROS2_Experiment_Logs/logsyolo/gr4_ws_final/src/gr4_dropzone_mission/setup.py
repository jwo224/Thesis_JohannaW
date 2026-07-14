from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'gr4_dropzone_mission'

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
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group4',
    maintainer_email='group4@example.com',
    description='Real robot dropzone markers and Nav2 command dispatcher.',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dropzone_mission = gr4_dropzone_mission.dropzone_mission:main',
        ],
    },
)
