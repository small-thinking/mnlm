from flask import Flask, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading

class FlaskNode(Node):
    def __init__(self):
        super().__init__('command_receiver_node')
        self.publisher_ = self.create_publisher(String, 'json_command_topic', 10)

    def publish_json_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)

def create_app(ros_node):
    app = Flask(__name__)

    @app.route('/command', methods=['POST'])
    def json_example():
        if request.is_json:
            content = request.get_json()
            command_str = json.dumps(content)
            # log the JSON string received
            ros_node.get_logger().info('Received JSON command: %s' % command_str)
            ros_node.publish_json_command(command_str)
            return 'JSON command received and published!', 200
        else:
            return 'Request was not JSON', 400

    return app

def main(args=None):
    rclpy.init(args=args)
    ros_node = FlaskNode()
    app = create_app(ros_node)

    # Running Flask in a separate thread
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000), daemon=True).start()
    
    rclpy.spin(ros_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
