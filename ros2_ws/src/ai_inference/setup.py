from setuptools import find_packages, setup
import os  # 👈 이것이 추가되었는지 확인
from glob import glob # 👈 이것이 추가되었는지 확인

package_name = 'ai_inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 💡 이 부분이 추가되어야 합니다.
        # Launch 파일이 'launch' 폴더에 있고, 이름이 '*.launch.py' 형식이라고 가정합니다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='11306260+liangfuyuan@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bev_creator = ai_inference.bev_creator_node:main',
            'bev_buffer = ai_inference.bev_buffer_node:main',
            'onnx_future_predictor = ai_inference.onnx_future_predictor_node:main',
            'heatmap_bias = ai_inference.heatmap_bias_node:main',
            'riskmap_markerarray = ai_inference.riskmap_markerarray:main',
            'mega_node = ai_inference.mega_node:main',
        ],
    },
)
