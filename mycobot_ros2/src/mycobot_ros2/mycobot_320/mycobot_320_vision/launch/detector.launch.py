# mycobot_320_vision/launch/detector.launch.py
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # 런치 인자 선언: detect_mode (기본 detect_only)
    detect_mode_arg = DeclareLaunchArgument(
        'detect_mode',
        default_value='detect_only',
        description="Detector mode: 'detect_only' or 'detect_and_classify'"
    )

    # (옵션) 카메라 인덱스도 런치 인자로 노출하고 싶다면 추가
    cam_index_arg = DeclareLaunchArgument(
        'cam_index',
        default_value='0',
        description='OpenCV camera index (int)'
    )

    # Node에 파라미터 전달
    detector_node = Node(
        package='mycobot_320_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{
            'detect_mode': LaunchConfiguration('detect_mode'),
            # detector.py에서 C.CAM_INDEX를 기본 쓰지만,
            # rclpy 파라미터도 쓰고 싶다면 detector.py에 declare_parameter/get_parameter 추가
            'camera_index': LaunchConfiguration('cam_index')
        }]
    )

    return LaunchDescription([
        detect_mode_arg,
        cam_index_arg,
        detector_node,
    ])
