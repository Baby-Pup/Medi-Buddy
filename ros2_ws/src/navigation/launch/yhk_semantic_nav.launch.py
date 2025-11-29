#!/usr/bin/env python3
"""
semantic_nav.launch.py

한 번에 다음 노드들을 실행:
1. Nav2 Bringup (navigation.launch.py)
2. RViz (rviz_navigation.launch.py)
3. Semantic Router Node (yhk_semantic_router_node.py)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # === 인자 정의 ===
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/ros2_ws/src/slam/maps/map_01.yaml',
        description='Path to map yaml file'
    )

    map_path = LaunchConfiguration('map')

    # === 경로 설정 ===
    pkg_launch_dir = os.path.join(
        os.path.dirname(__file__)  # ~/ros2_ws/src/navigation/launch
    )

    # === Nav2 bringup (지도 퍼블리시 + 경로계획) ===
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch_dir, 'navigation.launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )

    # # === RViz (시각화) ===
    # rviz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_launch_dir, 'rviz_navigation.launch.py')
    #     ),
    #     launch_arguments={'map': map_path}.items()
    # )

    # === Semantic Router Node ===
    semantic_router = Node(
        package='navigation',
        executable='semantic_router_node',  # setup.py entry_points와 동일해야 함
        name='semantic_router',
        output='screen',
        parameters=[{
            'poi_yaml_path': '/home/ubuntu/ros2_ws/src/navigation/config/poi_map.yaml'
        }]
    )

    # === LaunchDescription 반환 ===
    return LaunchDescription([
        map_arg,
        nav_bringup,
        # rviz,
        semantic_router
    ])
