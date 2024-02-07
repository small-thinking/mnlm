import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import json


class CommandDispatcherNode(Node):
    def __init__(self):
        super().__init__("command_dispatcher_node")
        self.node_name = self.get_name()
        self.subscription = self.create_subscription(
            String, "json_command_topic", self._json_command_callback, 10
        )
        # Setup publish to servos
        self.servo0_pub = self.create_publisher(Float64, '/servo0/control', 10)

    def _json_command_callback(self, msg):
        """
        This function is called every time a message is received on the json_command_topic
        ."""
        try:
            command_data = json.loads(msg.data)
            # Process the command_data to control the joint
            self._process_command(command_data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received")

    def _process_command(self, command_data):
        # Example: command_data = {"action": "move", "joint": "joint1", "value": 1.0}
        action = command_data.get("action")
        joint_name = command_data.get("joint")
        value = command_data.get("value")

        if action == "move":
            # Implement the logic to move the specified joint to the given value
            # This will depend on your specific robot and control system
            self._move_joint(joint_name, value)
        else:
            self.get_logger().warn(f"Unknown action: {action}")

    def _move_joint(self, joint_name, value):
        if joint_name == 'servo0':
            control_msg = Float64()
            control_msg.data = value
            self.servo0_pub.publish(control_msg)
            self.get_logger().info(f'{self.node_name} Moving {joint_name} to {value}')
        else:
            self.get_logger().warn(f'{self.node_name} Unknown joint: {joint_name}')


def main(args=None):
    rclpy.init(args=args)
    node = CommandDispatcherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
