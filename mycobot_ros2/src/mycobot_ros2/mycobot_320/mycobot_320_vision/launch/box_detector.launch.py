from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mycobot_320_vision',
            executable='box_detector',
            name='box_detector_node',
            output='screen',
            parameters=[
                {'camera_index': 0},
                {'min_area': 1000},
                {'calc_roll': True},
                {'frame_rate': 30.0},
                # 필요시 HSV 범위 조정
                {'lower_green': [35, 50, 50]},
                {'upper_green': [85, 255, 255]},
                {'lower_white': [0, 0, 250]},
                {'upper_white': [180, 50, 255]},
            ],
        )
    ])
