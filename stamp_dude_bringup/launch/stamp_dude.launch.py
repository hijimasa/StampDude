from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stamp_dude_node',
            executable='stamp_dude_node',
            name='stamp_dude_node',
            parameters=[
                {'topic_name': '/cmd_vel'}  # 使用したいトピック名に置き換え
            ],
            output='screen'
        ),
    ])
