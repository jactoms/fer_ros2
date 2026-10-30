from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fer_track_hand'

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
    maintainer='Tom Jacob',
    maintainer_email='jacobtom99rp@gmail.com',
    description='Hand tracker: moves the tracker cube of fer_hand_mjc to the hand.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'track_hand_node = fer_track_hand.track_hand_node:main',
        ],
    },
)
