from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    http_host = LaunchConfiguration('http_host')
    http_port = LaunchConfiguration('http_port')
    ws_host = LaunchConfiguration('ws_host')
    ws_port = LaunchConfiguration('ws_port')
    ws_path = LaunchConfiguration('ws_path')
    topic = LaunchConfiguration('topic')

    bridge = Node(
        package='tmr4243_utilities',
        executable='web_joystick_node.py',
        name='web_joystick',
        output='screen',
        arguments=[
            '--http-host', http_host,
            '--http-port', http_port,
            '--ws-host', ws_host,
            '--ws-port', ws_port,
            '--ws-path', ws_path,
            '--topic', topic,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'http_host',
            default_value='127.0.0.1',
            description='HTTP bind host (use 0.0.0.0 to access from other devices)',
        ),
        DeclareLaunchArgument('http_port', default_value='8000', description='HTTP port'),
        DeclareLaunchArgument(
            'ws_host',
            default_value='127.0.0.1',
            description='WebSocket bind host (use 0.0.0.0 to access from other devices)',
        ),
        DeclareLaunchArgument('ws_port', default_value='8765', description='WebSocket port'),
        DeclareLaunchArgument('ws_path', default_value='/ws', description='WebSocket path'),
        DeclareLaunchArgument('topic', default_value='/joy', description='ROS2 Joy topic'),
        bridge,
    ])
