import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CommandDispatcherNode(Node):
    def __init__(self):
        super().__init__('command_dispatcher_node')
        self.node_name = self.get_name()
        self.subscription = self.create_subscription(
            String, 'json_command_topic', self.json_command_callback, 10)
        # TODO: Add any publishers or clients needed to control the joint

    def json_command_callback(self, msg):
        try:
            command_data = json.loads(msg.data)
            self.get_logger().info(f'{self.node_name} Received command: {command_data}')
            # Process the command_data to control the joint
            self.process_command(command_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON received')

    def process_command(self, command_data):
        # Example: command_data = {"action": "move", "joint": "joint1", "value": 1.0}
        action = command_data.get("action")
        joint_name = command_data.get("joint")
        value = command_data.get("value")

        if action == "move":
            # Implement the logic to move the specified joint to the given value
            # This will depend on your specific robot and control system
            self.move_joint(joint_name, value)
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def move_joint(self, joint_name, value):
        # Logic to move the joint
        # This is highly dependent on your setup (simulation, real hardware, etc.)
        self.get_logger().info(f'{self.node_name} Moving {joint_name} to {value}')
        # Here, integrate with your robot arm's control interface

def main(args=None):
    rclpy.init(args=args)
    node = CommandDispatcherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
