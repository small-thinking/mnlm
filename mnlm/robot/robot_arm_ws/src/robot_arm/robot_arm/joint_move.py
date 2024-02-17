#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')
        joint_names = ["servo0", "servo1"]
        # joint_names = ["servo1"]
        self.declare_parameter('joint_names', joint_names)  # Default joint names
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        
        self.action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal_and_wait(self, positions, velocities, time_from_start_sec):
        self.get_logger().info('Sending new goal to the action server...')
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions  # Specify positions for each joint
        point.velocities = velocities  # Specify velocities for each joint
        point.time_from_start = Duration(sec=time_from_start_sec, nanosec=0)
        trajectory.points.append(point)

        goal_msg.trajectory = trajectory

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return False

        self.get_logger().info('Goal accepted :)')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        if result:
            self.get_logger().info(f'Result: {result}')
            
        self.action_client.wait_for_server()
        return True

    def move_robot_arm(self):
        # Example movement: Move to initial position, then to another position
        if self.send_goal_and_wait(positions=[1.5, 1.5], velocities=[0.5, 0.5], time_from_start_sec=3):
            self.send_goal_and_wait(positions=[0.0, 0.0], velocities=[0.5, 0.5], time_from_start_sec=3)

    def _feedback_callback(self, feedback_msg):
        actual_positions = feedback_msg.feedback.actual.positions
        actual_velocities = feedback_msg.feedback.actual.velocities
        self.get_logger().info(f'Received feedback: positions: {actual_positions}, velocities: {actual_velocities}')

def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient()
    try:
        action_client.move_robot_arm()
    except Exception as e:
        action_client.get_logger().error(f'Encountered an error: {e}')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()
    action_client.get_logger().info('action_client destroyed and node shut down.')

if __name__ == '__main__':
    main()
