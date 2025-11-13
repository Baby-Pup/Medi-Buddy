from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'hospital_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 (현재는 없지만, 확장성을 위해)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # [중요] data 폴더의 JSON 및 CSV 파일 설치
        (os.path.join('share', package_name, 'data'), glob(os.path.join('data', '*.json'))),
        (os.path.join('share', package_name, 'data'), glob(os.path.join('data', '*.csv'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # <-- 수정
    maintainer_email='your_email@todo.todo', # <-- 수정
    description='RAG-based Hospital Robot Service', # <-- 수정
    license='apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'rag_server'라는 실행 이름으로 rag_server_node.py의 main 함수 연결
            'rag_server = hospital_robot_pkg.rag_server_node:main',
            # 'robot_client'라는 실행 이름으로 robot_client_node.py의 main 함수 연결
            'robot_client = hospital_robot_pkg.robot_client_node:main',
        ],
    },
)