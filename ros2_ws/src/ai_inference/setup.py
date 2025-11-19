from setuptools import find_packages, setup

package_name = 'ai_inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
