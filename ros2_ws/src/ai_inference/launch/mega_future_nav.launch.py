from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    navigation_pkg = get_package_share_directory('navigation')

    # 나중에 이건 다른 걸로 바꿔줘야 함. yhk_semantic_nav.launch.py 이걸로.
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg, 'launch', 'include', 'bringup.launch.py')
        ),
        launch_arguments={
            'map': os.path.join(navigation_pkg, 'maps', 'map_01.yaml'),
            'params_file': os.path.join(navigation_pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        # 1) BEV Creator
        Node(
            package='future_prediction_pkg',
            executable='bev_creator',
            name='bev_creator',
            output='screen'
        ),

        # 2) BEV Buffer
        Node(
            package='future_prediction_pkg',
            executable='bev_buffer',
            name='bev_buffer',
            output='screen'
        ),

        # 3) Hailo 미래 예측기
        Node(
            package='future_prediction_pkg',
            executable='future_predictor_hailo',
            name='hailo_future_predictor',
            output='screen'
        ),

        # 4) Heatmap Bias (risk_map/omega_weights)
        Node(
            package='future_prediction_pkg',
            executable='future_heatmap_bias',
            name='future_heatmap_bias',
            output='screen'
        ),

        # 5) Nav2 Bringup
        bringup_launch
    ])
