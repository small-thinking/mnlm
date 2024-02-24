import json
import requests
import os

def send_commands_to_service(json_file_path, service_url):
    # Load the JSON commands from the specified file
    with open(json_file_path, 'r') as file:
        commands = json.load(file)

    # Send the JSON commands to the HTTP service
    response = requests.post(service_url, json=commands)

    # Check the response status
    if response.status_code == 200:
        print("Commands sent successfully.")
        print("Response:", response.json())
    else:
        print("Failed to send commands.")
        print("Status Code:", response.status_code)
        print("Response:", response.text)

if __name__ == "__main__":
    json_file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "knowledge/dummy_command.json")
    service_url = "http://0.0.0.0:5678/robot_command"

    send_commands_to_service(json_file_path, service_url)
