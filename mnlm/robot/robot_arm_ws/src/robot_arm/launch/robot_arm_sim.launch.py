from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    command_receiver_node = Node(
        package='robot_arm',
        executable='command_receiver_node',
        name='command_receiver_node'
    )

    command_dispatcher_node = Node(
        package='robot_arm',
        executable='command_dispatcher_node',
        name='command_dispatcher_node'
    )

    return LaunchDescription([
        command_receiver_node,
        command_dispatcher_node
    ])
