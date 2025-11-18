import os
from glob import glob
from setuptools import setup # 👈 1. find_packages 임포트

package_name = 'hailo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # 👈 2. 이 줄을 주석 처리 (또는 삭제)
    #packages=find_packages(exclude=['test']), # 👈 3. 이 줄로 변경
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
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bev_creator = hailo.bev_creator:main',
            'bev_buffer = hailo.bev_buffer:main',
            'bev_reciever = hailo.bev_reciever:main',
        ],
    },
)