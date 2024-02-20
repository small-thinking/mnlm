#!/usr/bin/env python3
import json
import traceback
from std_msgs.msg import String
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from typing import List
from sensor_msgs.msg import JointState


class JointMoverNode(Node):
    def __init__(self):
        super().__init__("joint_mover_node")
        self.num_joints = 5
        joint_names = []
        for i in range(self.num_joints):
            joint_names.append("servo" + str(i))
        # Add left and right gripper joints
        self.num_gripper_fingers = 2
        joint_names.append("left_gripper_joint")
        joint_names.append("right_gripper_joint")
        self.declare_parameter("joint_names", joint_names)  # Default joint names
        self.joint_names = (
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        # Used to receive joint states from the robot.
        self.latest_joint_state = None
        self.joint_state_subscription = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)
        # Used to receive commands from the CommandDispatcherNode.
        self.subscription = self.create_subscription(
            String,
            "/joint_commands",
            self._joint_command_callback,
            10,
        )
        # Used to send commands to the robot.
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        
    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg
            
    def _get_current_joint_positions(self, joint_names: List[str]):
        """Return the positions of the specified joints using the latest joint state message."""
        if self.latest_joint_state is None:
            self.get_logger().error('No joint state message received yet.')
            return None
        
        positions = []
        for name in joint_names:
            try:
                index = self.latest_joint_state.name.index(name)
                positions.append(self.latest_joint_state.position[index])
            except ValueError:
                self.get_logger().error(f'Joint {name} not found in the latest joint state message.')
                return None
        return positions
        
    def _joint_command_callback(self, msg: String):
        self.get_logger().error(f"Received joint command: {msg.data}")
        command_data = json.loads(msg.data)
        
        joint_id = command_data["id"]
        angle_in_degrees = command_data["angle"]
        time_in_seconds = command_data["time"] / 1000.0  # Convert milliseconds to seconds

        # Read the current position from /joint_states topic
        positions = self._get_current_joint_positions(self.joint_names)
        self.get_logger().info(f"Current joint positions: {positions}")
        velocities = [0.0] * (self.num_joints + self.num_gripper_fingers)
        
        # Update the angle for the specified joint, note the command is in degree and the position is in radians
        if "servo" in joint_id:
            target_servo_index = int(joint_id[-1])
            target_position = angle_in_degrees * (3.14159 / 180.0)  # Convert degrees to radians
            positions[target_servo_index] = target_position
        
            # Only set the velocity of the target joint as 0.5
            velocities[target_servo_index] = 0.5
            # Send the command to the action server
            response = self.send_goal_and_wait(positions=positions, velocities=velocities, time_from_start_sec=3)
            if response:
                self.get_logger().info(f"Successfully moved joint {joint_id} to {angle_in_degrees} degrees.")
        else:
            self.get_logger().error(f"Invalid joint ID: {joint_id}")
        

    def send_goal_and_wait(self, positions: List[float], velocities: List[float], time_from_start_sec: int) -> bool:
        self.get_logger().info("Sending new goal to the action server...")
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()

        trajectory.joint_names = self.joint_names
        # Construct the trajectory point to send to the action server.
        point = JointTrajectoryPoint()
        point.positions = positions 
        point.velocities = velocities
        timeout_sec_full = int(time_from_start_sec)
        point.time_from_start = Duration(sec=timeout_sec_full, nanosec=0)
        trajectory.points.append(point)

        goal_msg.trajectory = trajectory

        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server not available after waiting')
            return False
        else:
            self.get_logger().info("Action server available.")
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        # rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=3)
        self.get_logger().info("Goal sent.")
        return True

    def _feedback_callback(self, feedback_msg):
        actual_positions = feedback_msg.feedback.actual.positions
        actual_velocities = feedback_msg.feedback.actual.velocities
        self.get_logger().info(
            f"Received feedback: positions: {actual_positions}, velocities: {actual_velocities}"
        )


def main(args=None):
    rclpy.init(args=args)
    joint_mover_node = JointMoverNode()
    rclpy.spin(joint_mover_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
