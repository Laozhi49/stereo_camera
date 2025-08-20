from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_camera',
            executable='stereo_camera_node',
            name='stereo_camera',
            parameters=[{'show_preview': True}],
            output='screen'
        )
    ])