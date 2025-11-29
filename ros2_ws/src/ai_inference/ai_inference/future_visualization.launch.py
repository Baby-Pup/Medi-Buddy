from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_pkg',
            executable='riskmap_markerarray',
            name='riskmap_markerarray'
        ),
        Node(
            package='your_pkg',
            executable='future_animation',
            name='future_animation'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/ubuntu/rviz/future_vis.rviz']
        )
    ])
