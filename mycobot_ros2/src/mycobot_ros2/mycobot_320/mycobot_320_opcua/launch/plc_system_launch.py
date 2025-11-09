from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'mycobot_320_opcua'

    opcua_node = Node(
        package=package_name,
        executable='opcua_server',
        name='opcua_server',
        output='screen'
    )

    m0010_node = Node(
        package=package_name,
        executable='m0010_listener',
        name='m0010_listener',
        output='screen'
    )

    m0020_node = Node(
        package=package_name,
        executable='m0020_listener',
        name='m0020_listener',
        output='screen'
    )

    return LaunchDescription([
        opcua_node,
        m0010_node,
        m0020_node
    ])
