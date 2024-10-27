from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='twist_stamped_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='stamp_dude',
                plugin='stamp_dude::TwistToTwistStamped',
                name='twist_to_twist_stamped_node'
            ),
        ],
        output='screen'
    )

    return LaunchDescription([container])
