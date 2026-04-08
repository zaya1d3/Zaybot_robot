from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zaybot_teleop_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zayan Zamir',
    maintainer_email='zayanabdullahzamir@gmail.com',
    author='Zayan Zamir',
    author_email='zayanabdullahzamir@gmail.com',
    description='Modular teleoperation GUI for ROS 2 with multiple control modes. '
                'Initially developed for the Zaybot robot but adaptable to any robot '
                'using geometry_msgs/Twist.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_gui = zaybot_teleop_gui.main:main',
        ],
    },
)

