import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    # ✅ Vision 노드 실행 여부
    start_vision_arg = DeclareLaunchArgument(
        'start_vision',
        default_value='false',
        description='Start vision detector node together'
    )
    start_vision = LaunchConfiguration('start_vision')

    # ✅ YOLO 모드 설정 (detect_only / detect_and_classify)
    detect_mode_arg = DeclareLaunchArgument(
        'detect_mode',
        default_value='detect_only',
        description='Choose vision detection mode: detect_only or detect_and_classify'
    )
    detect_mode = LaunchConfiguration('detect_mode')

    # ✅ 테스트 모드 추가 (True면 내부 테스트 시퀀스 실행)
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Enable internal test mode for classify_control (bypass external sequence)'
    )
    test_mode = LaunchConfiguration('test_mode')

    # ✅ listen_real_service (서버)
    service_node = Node(
        package='mycobot_320',
        executable='listen_real_service',
        name='listen_real_service',
        output='screen',
        parameters=[{'port': '/dev/ttyACM0', 'baud': 115200}],
        condition=UnlessCondition(test_mode)
    )

    # ✅ classify_control (클라이언트)
    control_node = Node(
        package='mycobot_320_ctrl',
        executable='classify_control',
        name='classify_control',
        output='screen',
        parameters=[{'detect_mode': detect_mode}] ## 이 parameters를 detect_mode로 변경
    )

    # ✅ Vision(detector) 노드 (옵션)
    vision_node = Node(
        package='mycobot_320_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        condition=IfCondition(start_vision),
        parameters=[{'detect_mode': detect_mode}]
    )

    ld = LaunchDescription()
    ld.add_action(start_vision_arg)
    ld.add_action(detect_mode_arg)
    ld.add_action(test_mode_arg)
    ld.add_action(service_node)
    ld.add_action(control_node)
    ld.add_action(vision_node)

    return ld
