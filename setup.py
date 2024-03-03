from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'piagv'

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
    maintainer='Linxuan Biao',
    maintainer_email='l.biao@liverpool.ac.uk',
    description='',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_interface = piagv.vehicle_interface:main',
            'world_to_odom_publisher = piagv.world_to_odom_publisher:main',
            'simple_planner = piagv.simple_planner:main',
            'speed_limiter = piagv.speed_limiter:main',
        ],
    },
)
