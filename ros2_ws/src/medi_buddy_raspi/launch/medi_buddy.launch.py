from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    navigation_package_path = get_package_share_directory('navigation')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/ros2_ws/src/slam/maps/map_hj.yaml',
        description='Map yaml file'
    )

    map_path = LaunchConfiguration('map')


    semantic_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/yhk_semantic_nav.launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )
    
    # Camera publisher
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

    return LaunchDescription([
        map_arg,
        camera_publisher,
        qrcode_publisher,
        detection_subscriber,
        audio_publisher,
        audio_reciever,
        # semantic_nav_launch
    ])
