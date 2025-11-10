import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 설정
    package_name = 'mycobot_opcua_control'
    
    # 2. MyCobot Controller 노드 정의
    mycobot_controller_node = Node(
        package=package_name,
        executable='mycobot_controller.py', # 💡 실행할 Python 파일명
        name='mycobot_opcua_controller',    # 💡 ROS 2에서 노드의 이름
        output='screen',                    # 터미널에 노드 출력 표시
        emulate_tty=True,                   # 노드의 출력을 올바르게 표시
        # parameters=[{'param_name': 'param_value'}] # 필요시 파라미터 추가 가능
    )

    # 3. 런치 디스크립션 반환
    return LaunchDescription([
        mycobot_controller_node
    ])