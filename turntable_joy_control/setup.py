import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turntable_joy_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Devansh A. Bajwala',
    maintainer_email='db7529@uw.edu',
    description='Jog control for turntable via keyboard or Xbox controller',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'turntable_joy_node = turntable_joy_control.turntable_joy_node:main',
            'turntable_keyboard_node = turntable_joy_control.turntable_keyboard_node:main',
        ],
    },
)
