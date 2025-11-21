from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # 'navigation' 패키지 경로를 가져옵니다.
    navigation_pkg = get_package_share_directory('navigation')
    
    # 💡 1. Nav2 Bringup (LiDAR, Odometry, TF, Nav2 핵심 노드 모두 포함)
    # Nav2 bringup이 LiDAR와 Odom 드라이버를 모두 실행한다고 가정합니다.
    nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # navigation 패키지 내부의 navigation.launch.py를 포함합니다.
            os.path.join(navigation_pkg, 'launch', 'navigation.launch.py') 
        ),
        launch_arguments={
            # 이 설정 파일은 DWBLocalPlanner 및 OmegaCritic을 포함하고 있습니다.
            'params_file': os.path.join(navigation_pkg, 'config', 'nav2_params.yaml'),
            # 필요한 경우 맵 파일 경로도 지정 (nav2_params.yaml 내부에 이미 있을 수도 있습니다)
            'map': os.path.join(navigation_pkg, 'maps', 'map_01.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items()
    )

    # 💡 2. RViz 실행 (Nav2의 RViz 파일 포함)
    # 기존의 ExecuteProcess 대신 Nav2가 제공하는 RViz Launch 파일을 포함합니다.
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg, 'launch', 'rviz_navigation.launch.py')
        ),
        launch_arguments={
            'map': os.path.join(navigation_pkg, 'maps', 'map_01.yaml')
        }.items()
    )

    return LaunchDescription([
        # 1) BEV Creator
        Node(
            package='ai_inference',
            executable='bev_creator',
            name='bev_creator',
            output='screen'
        ),

        # 2) BEV Buffer
        Node(
            package='ai_inference',
            executable='bev_buffer',
            name='bev_buffer',
            output='screen'
        ),

        # 3) Hailo 미래 예측기
        Node(
            package='ai_inference',
            executable='onnx_future_predictor',
            name='onnx_future_predictor',
            output='screen'
        ),

        # 4) Heatmap Bias (risk_map/omega_weights)
        Node(
            package='ai_inference',
            executable='heatmap_bias',
            name='heatmap_bias',
            output='screen'
        ),

        Node(
            package='ai_inference',
            executable='riskmap_markerarray',
            name='riskmap_markerarray',
            output='screen'
        ),        

        # 5) Nav2 Bringup (LiDAR, Odom 포함 가정)
        nav_bringup_launch,

        # 6) RViz (시각화)
        rviz_launch
    ])