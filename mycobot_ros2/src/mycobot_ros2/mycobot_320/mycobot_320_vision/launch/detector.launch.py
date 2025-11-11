from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mycobot_320_vision',
            executable='detector_node',
            name='detector_node',
            output='screen'
        ),
    ])

