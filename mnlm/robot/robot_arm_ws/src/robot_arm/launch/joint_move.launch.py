import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_arm",
                executable="joint_move_node",
                output="screen",
            ),
        ]
    )
