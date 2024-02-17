import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path to the XACRO file
    pkg_share = get_package_share_directory('robot_arm')
    # xacro_file = os.path.join(pkg_share, 'models', 'example_robot.urdf.xacro')
    xacro_file = os.path.join(pkg_share, 'models', 'arm.xacro')
    
    # Convert XACRO to URDF
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()
    
    # urdf_file_path = os.path.join(pkg_share, 'models', 'arm.urdf')
    # # Load the URDF file content
    # with open(urdf_file_path, 'r') as urdf_file:
    #     robot_description = urdf_file.read()

    # Node to publish the robot_description to the parameter server
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_arm.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz2
    ])
