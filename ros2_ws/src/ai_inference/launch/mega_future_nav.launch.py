from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # # 'navigation' 패키지 경로를 가져옵니다.
    # navigation_pkg = get_package_share_directory('navigation')
    
    # # 💡 1. Nav2 Bringup (LiDAR, Odometry, TF, Nav2 핵심 노드 모두 포함)
    # # Nav2 bringup이 LiDAR와 Odom 드라이버를 모두 실행한다고 가정합니다.
    # nav_bringup_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(navigation_pkg, 'launch', 'navigation.launch.py') 
    #     ),
    #     launch_arguments={
    #         'params_file': os.path.join(navigation_pkg, 'config', 'nav2_params.yaml'),
    #         # 'map': os.path.join(navigation_pkg, 'maps', 'map_01.yaml'),
    #         # ⬇⬇ 이렇게 바꾸기 (확장자 제거)
    #         'map': 'map_hk',
    #         'use_sim_time': 'false',
    #         'autostart': 'true',
    #     }.items()
    # )

    # # 💡 2. RViz 실행 (Nav2의 RViz 파일 포함)
    # # 기존의 ExecuteProcess 대신 Nav2가 제공하는 RViz Launch 파일을 포함합니다.
    # rviz_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(navigation_pkg, 'launch', 'rviz_navigation.launch.py')
    #     ),
    #     launch_arguments={
    #         # 'map': os.path.join(navigation_pkg, 'maps', 'map_01.yaml')
    #         'map': 'map_hk'
    #     }.items()
    # )

    return LaunchDescription([
        # 1) BEV Creator
        Node(
                package='ai_inference',
                executable='mega_node',
                name='mega_node',
                output='screen'
            ),

        # 5) Nav2 Bringup (LiDAR, Odom 포함 가정)
        # nav_bringup_launch,

        # 6) RViz (시각화)
        # rviz_launch
    ])