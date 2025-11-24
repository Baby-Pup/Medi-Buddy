import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'medi_buddy_raspi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),   
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
        (os.path.join('share', package_name, 'launch', 'include'), glob(os.path.join('launch', 'include', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='Raspi - Computer Communicating ROS2 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'audio_publisher = medi_buddy_raspi.audio_publisher:main',
            'audio_reciever = medi_buddy_raspi.audio_reciever:main',
            'camera_publisher = medi_buddy_raspi.camera_publisher:main',
        ],
    },
)
