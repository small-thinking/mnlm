import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from typing import Any, List
import json


class CommandDispatcherNode(Node):
    def __init__(self):
        super().__init__("command_dispatcher_node")
        self.node_name = self.get_name()
        self.num_joints = 5
        self.subscription = self.create_subscription(
            String, "json_command_topic", self._json_command_callback, 10
        )
        # Setup publish to servos
        self.servo0_pub = self.create_publisher(Float64, "/servo0/control", 10)

    def _json_command_callback(self, msg):
        """This function is called every time a message is received on the json_command_topic.
        An example input is:
        {
            "operations": [
                {
                    "operation": "move_single_servo",
                    "parameters": {"id": 1, "angle": 60, "time": 500}
                },
                {
                    "operation": "set_rgb_light",
                    "parameters": {"R": 255, "G": 0, "B": 0}
                },
                {
                    "operation": "move_single_servo",
                    "parameters": {"id": 1, "angle": 90, "time": 500}
                }
            ]
        }
        """
        try:
            self.get_logger().info(f"Dispatching command: {msg.data}")
            command_data = json.loads(msg.data)
            if "operations" in command_data:
                # Process the command_data to control the joint
                self._process_command(command_data["operations"])
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received")

    def _process_command(self, operations: List[Any]):
        """Parse the incoming command and take the appropriate action.
        """
        # Example: command_data = {"action": "move", "joint": "joint1", "value": 1.0}
        for operation in operations:
            func_name = operation.get("operation", "")
            parameters: Dict[str, Any] = operation.get("parameters", {})

            if func_name == "move_single_servo":
                self._move_single_joint(parameters.get("id", 0), parameters.get("angle", 0), parameters.get("time", 0))
            elif func_name == "set_rgb_light":  # set the RGB light
                pass
            else:
                self.get_logger().warn(f"Unknown action: {operation}")

    def _move_single_joint(self, id: int, angle: float, time: int):
        if 0 <= id <= self.num_joints:
            control_msg = Float64()
            # control_msg.data = value
            # self.servo0_pub.publish(control_msg)
            self.get_logger().error(f"{self.node_name} Moving {id} to {angle}")
        else:
            self.get_logger().warn(f"{self.node_name} Unknown joint: {id}")


def main(args=None):
    rclpy.init(args=args)
    node = CommandDispatcherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
