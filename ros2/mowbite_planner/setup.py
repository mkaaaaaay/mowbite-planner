import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mowbite_planner'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    # (contract.json: in the package, MowBite OS reads it there, and in share/ as before)
    package_data={package_name: ['contract.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', package_name + '/contract.json']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'tools'), glob('tools/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkaaaaaay',
    maintainer_email='mkaaaaaay@users.noreply.github.com',
    description='MowBite Planner: coverage path planner for robot mowers (perimeter loops, lanes, gentle turns)',
    license='GPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mowbite_planner_node = mowbite_planner.node:main',
            'mowbite_planner_mqtt = mowbite_planner.mqtt_service:main',
        ],
    },
)
