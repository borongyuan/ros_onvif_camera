from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='onvif_camera',
            executable='onvif_camera',
            parameters=[
                {'host': '192.168.1.64'},
                {'port': 80},
                {'user': 'admin'},
                {'passwd': 'hik12345'},
            ],
            output='screen',
        )
    ])
