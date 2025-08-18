from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    manager = ComposableNodeContainer(
        name='serial_manager',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='serial_manager_pkg',
                plugin='serial_manager::SerialManager',
                name='serial_manager',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )
    
    listener = Node(
        package='serial_user_pkg',
        executable='serial_listener',
        name='listener_node',
        namespace='',
        output='screen',
    )

    talker = Node(
        package='serial_user_pkg',
        executable='serial_talker',
        name='talker_node',
        namespace='',
        output='screen',
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace='',
        output='screen',
    )

    return LaunchDescription([manager, listener, talker, joy])