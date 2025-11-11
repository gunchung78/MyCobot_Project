import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():

    # ✅ 런치 인자: vision(detector) 같이 띄울지 여부
    start_vision_arg = DeclareLaunchArgument(
        'start_vision',
        default_value='false',
        description='Start vision detector node together'
    )
    start_vision = LaunchConfiguration('start_vision')

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

    # ✅ vision(detector) 노드 (옵션)
    vision_node = Node(
        package='mycobot_320_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        condition=IfCondition(start_vision)
    )

    # LaunchDescription 구성
    ld = LaunchDescription()
    ld.add_action(start_vision_arg)
    ld.add_action(service_node)
    ld.add_action(control_node)
    ld.add_action(vision_node)

    return ld

