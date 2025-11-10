import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # listen_real_service 실행 (서버)
    service_node = Node(
        package='mycobot_320',
        executable='listen_real_service',
        name='listen_real_service',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200
        }]
    )

    # classify_control 실행 (클라이언트)
    control_node = Node(
        package='mycobot_320_ctrl',
        executable='classify_control',
        name='classify_control',
        output='screen',
    )

    return LaunchDescription([
        service_node,
        control_node
    ])
