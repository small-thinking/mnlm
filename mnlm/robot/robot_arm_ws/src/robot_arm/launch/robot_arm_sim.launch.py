import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    # The node to receive the http request externally for robot control.
    # command_receiver_node = Node(
    #     package="robot_arm",
    #     executable="command_receiver_node",
    #     name="command_receiver_node",
    # )
    # # WIP. The node to dispatch the received command to topics.
    # command_dispatcher_node = Node(
    #     package="robot_arm",
    #     executable="command_dispatcher_node",
    #     name="command_dispatcher_node",
    # )

    # Include the project folder
    pkg_robot_arm_gazebo = get_package_share_directory("robot_arm")
    world_file_path = os.path.join(pkg_robot_arm_gazebo, "models", "world.sdf")
    xacro_file_path = os.path.join(pkg_robot_arm_gazebo, "models", "arm.xacro")
    
    # Convert xacro to URDF
    robot_description_content = Command([
        "xacro ", xacro_file_path
    ])
    
    # Create a temporary file to store the converted URDF
    temp_urdf_file = tempfile.NamedTemporaryFile(delete=True, mode='w', suffix='.urdf')
    urdf_file_path = temp_urdf_file.name

    # Convert XACRO to URDF and write to the temporary file
    command = f"xacro {xacro_file_path} > {urdf_file_path}"
    convert_xacro_to_urdf = ExecuteProcess(
        cmd=[command],
        shell=True,
        output='screen',
    )

    # Start Ignition Gazebo with an empty world
    start_gazebo_cmd = ExecuteProcess(
        cmd=["ign", "gazebo", world_file_path, "-v", "4"], output="screen"
    )

    # Spawn the URDF model into Gazebo
    spawn_model_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_ign_gazebo",
            "create",
            "-file",
            urdf_file_path,
            "-name",
            "robot_arm",
            # Make the base on the floor
            "-z",
            "0.2",
        ],
        output="screen",
    )

    # Start the robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    # Start the ros2_control system for joint state broadcasting
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Start the controller for joint trajectory control
    robot_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Start them all
    return LaunchDescription(
        [
            convert_xacro_to_urdf,
            start_gazebo_cmd,
            spawn_model_cmd,
            robot_state_publisher_node,
            joint_state_broadcaster_node,
            robot_controller_spawner_node,
            # command_receiver_node,
        ]
    )
