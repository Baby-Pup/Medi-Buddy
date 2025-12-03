from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    navigation_package_path = get_package_share_directory('navigation')

    # ---------------------------------------------------------
    # ⚠️ [수정 1] 본인이 만든 파이썬 파일(가드)의 절대 경로를 입력하세요
    # ---------------------------------------------------------
    ai_script_path = '/home/ubuntu/ros2_ws/src/navigation/scripts/hospital_robot_main.py' 
    # (파일 이름이 hospital_guard.py가 아니라면 수정하세요)

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/ros2_ws/src/slam/maps/map_medibuddy.yaml',
        description='Map yaml file'
    )

    map_path = LaunchConfiguration('map')
    
    # Camera publisher (하드웨어 드라이버)
    camera_publisher = Node(
            package='medi_buddy_raspi',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        )

    # Qrcode publisher
    qrcode_publisher = Node(
            package='medi_buddy_qrcode',
            executable='qrcode_publisher',
            name='qrcode_publisher',
            output='screen'
        )

    # Face Detection subscriber
    detection_subscriber = Node(
            package='medi_buddy_facedetection',
            executable='detection_subscriber',
            name='detection_subscriber',
            output='screen'
        )

    # Audio publisher
    audio_publisher = Node(
            package='medi_buddy_raspi',
            executable='audio_publisher',
            name='audio_publisher',
            output='screen'
        )

    # Audio reciever
    audio_reciever = Node(
            package='medi_buddy_raspi',
            executable='audio_reciever',
            name='audio_reciever',
            output='screen'
        )

    # Semantic router
    semantic_router = Node(
        package='navigation',
        executable='semantic_router_node',
        name='semantic_router',
        output='screen',
        parameters=[{
            'poi_yaml_path': '/home/ubuntu/ros2_ws/src/navigation/config/poi_map.yaml'
        }]
    )

    # Nav2 bringup launch
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/navigation.launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )

    # ---------------------------------------------------------
    # ⚠️ [수정 2] AI 가드 실행 설정 (TimerAction 사용)
    # Nav2와 카메라가 완전히 켜질 때까지 10초 기다렸다가 실행합니다.
    # ---------------------------------------------------------
    ai_guard_runner = TimerAction(
        period=10.0, 
        actions=[
            ExecuteProcess(
                cmd=['python3', ai_script_path],
                output='screen',
                name='hospital_safety_guard'
            )
        ]
    )

    return LaunchDescription([
        map_arg,
        camera_publisher,
        qrcode_publisher,
        detection_subscriber,
        audio_publisher,
        audio_reciever,
        semantic_router,
        nav_bringup,
        ai_guard_runner # 👈 [수정 3] 여기에 추가
    ])