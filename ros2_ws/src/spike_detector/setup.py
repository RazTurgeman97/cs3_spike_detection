from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spike_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Resource index for package discovery
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files - automatically includes all .launch.py files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Configuration files - includes all .yaml files for different sensitivity levels
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raz',
    maintainer_email='RazTurgeman97@gmail.com',
    description='ROS 2 package for detecting height spikes in trajectory data from handheld mobile scanners',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Main executable for the spike detector node
            'spike_detector_node = spike_detector.spike_detector_node:main',
            # Add analyzer executable
            'spike_analyzer = spike_detector.spike_analyzer:main',
        ],
    },
)
