from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    navigation_package_path = get_package_share_directory('navigation')

    ai_script_path = '/home/ubuntu/ros2_ws/src/navigation/scripts/hospital_robot_main.py'

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/ros2_ws/src/slam/maps/map_medibuddy.yaml',
        description='Map yaml file'
    )
    map_path = LaunchConfiguration('map')

    # 실행할 depth camera launch
    peripherals_package_path = get_package_share_directory('peripherals')

    depth_camera_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/include/ascamera.launch.py')
        ),
        launch_arguments={
            'depth_camera_name': 'depth_cam',
            'tf_prefix': '',
            'app': 'false'
        }.items()
    )

    camera_publisher = Node(
        package='medi_buddy_raspi',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen'
    )

    qrcode_publisher = Node(
        package='medi_buddy_qrcode',
        executable='qrcode_publisher',
        name='qrcode_publisher',
        output='screen'
    )

    detection_subscriber = Node(
        package='medi_buddy_facedetection',
        executable='detection_subscriber',
        name='detection_subscriber',
        output='screen'
    )

    audio_publisher = Node(
        package='medi_buddy_raspi',
        executable='audio_publisher',
        name='audio_publisher',
        output='screen'
    )

    audio_reciever = Node(
        package='medi_buddy_raspi',
        executable='audio_reciever',
        name='audio_reciever',
        output='screen'
    )

    semantic_router = Node(
        package='navigation',
        executable='semantic_router_node',
        name='semantic_router',
        output='screen',
        parameters=[{
            'poi_yaml_path': '/home/ubuntu/ros2_ws/src/navigation/config/poi_map.yaml'
        }]
    )

    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/navigation.launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )

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
        #depth_camera_launcher,  # ⬅ 추가된 실행 (peripherals depth camera)
        camera_publisher,
        qrcode_publisher,
        detection_subscriber,
        audio_publisher,
        audio_reciever,
        semantic_router,
        nav_bringup,
        # ai_guard_runner
    ])