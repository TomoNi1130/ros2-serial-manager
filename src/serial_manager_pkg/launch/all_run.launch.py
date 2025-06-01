from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    run = ComposableNodeContainer(
        name='serial_manager',
        namespace='UC',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='serial_manager_pkg',
                plugin='serial_manager::SerialManager',
                name='serial_manager',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),ComposableNode(
                package='ros2_controller_pkg',
                plugin='ros2_controller::ROS2Controller',
                name='ros2_controller',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),ComposableNode(
                package='drive_controller_pkg',
                plugin='drive_controller::DriveController',
                name='drive_controller',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{
                'auto_mode': False,
                'diff_num': 4
                }]
            ),ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ])
    return LaunchDescription([run])