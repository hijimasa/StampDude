import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='stamp_dude_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            parameters=[],
            composable_node_descriptions=[
                LoadComposableNode(
                    package='stamp_dude',
                    plugin='stamp_dude::StampDudeNode',
                    name='twist_stamper',
                    parameters=[],
                ),
                LoadComposableNode(
                    package='stamp_dude',
                    plugin='stamp_dude::StampDudeNode',
                    name='pose_stamper',
                    parameters=[],
                ),
            ],
        ),
    ])
