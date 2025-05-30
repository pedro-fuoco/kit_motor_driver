import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kit_motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedro-fuoco',
    maintainer_email='pedrofuoco6@gmail.com',
    description='ROS 2 motor driver for Kit de Robotica',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = kit_motor_driver.motor_node:main',
            'controller_node = kit_motor_driver.controller_node:main',
        ],
    },
)