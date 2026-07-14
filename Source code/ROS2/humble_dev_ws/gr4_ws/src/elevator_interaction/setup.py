from setuptools import find_packages, setup

package_name = 'elevator_interaction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (
            'share/' + package_name,
            ['package.xml', 'BAROMETER_README.md'],
        ),
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
            'elevator_node = elevator_interaction.elevator_node:main',
            (
                'esp32_elevator_client = '
                'elevator_interaction.esp32_elevator_client:main'
            ),
            (
                'ble_elevator_client = '
                'elevator_interaction.ble_elevator_client:main'
            ),
            (
                'esp32_serial_bridge = '
                'elevator_interaction.esp32_serial_bridge:main'
            ),
            (
                'elevator_csv_calibration_logger = '
                'elevator_interaction.elevator_csv_calibration_logger:main'
            ),
        ],
    },
)
