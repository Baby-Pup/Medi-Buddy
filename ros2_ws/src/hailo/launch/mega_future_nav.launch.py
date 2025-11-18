from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    navigation_package_path = get_package_share_directory('navigation')
    peripherals_package_path = get_package_share_directory('peripherals')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar.launch.py')),
        launch_arguments={
            'scan_topic': '/scan_raw',
            'scan_raw': '/scan_raw',
        }.items(),
    )

    return LaunchDescription([
        # 1) BEV Creator
        Node(
            package='hailo',
            executable='bev_creator',
            name='bev_creator',
            output='screen'
        ),

        # 2) BEV Buffer
        Node(
            package='hailo',
            executable='bev_buffer',
            name='bev_buffer',
            output='screen'
        ),

        # 3) BEV Reciever
        Node(
            package='hailo',
            executable='bev_reciever',
            name='bev_reciever',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['rviz2', 'rviz2', '-d', os.path.join(navigation_package_path, 'rviz/bev_only.rviz')],
            output='screen'
        ),

        lidar_launch
    ])
