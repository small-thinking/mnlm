import json
import os
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

import requests
from colorama import Fore
from dotenv import load_dotenv
from openai import OpenAI
from utils import Logger


class OperationSequenceGenerator:
    """Operation sequence generator that convert the operations to json."""

    def __init__(
        self,
        api_document_path: str,
        gpt_client: Optional[OpenAI] = None,
        logger: Optional[Logger] = None,
        verbose: bool = False,
    ):
        # Load api document from the file path. The content of the file will be used as part of prompt.
        load_dotenv()
        self.verbose = verbose
        self.logger = logger if logger else Logger(__file__)
        self.api_document = ""
        if os.path.exists(os.path.expanduser(api_document_path)):
            with open(os.path.expanduser(api_document_path), "r") as f:
                self.api_document = "".join(f.readlines())
            if self.verbose:
                self.logger.info(
                    f"Loaded the API doc for robot arm. Size: {len(self.api_document)}"
                )
        else:
            if self.verbose:
                self.logger.warning(
                    f"API document file {api_document_path} does not exist."
                )
        self.gpt_client = (
            OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
            if not gpt_client
            else gpt_client
        )
        self.max_tokens = 4000

    def generate_operation(self, operation_name: str, **kwargs) -> str:
        """Generate a JSON string representing a single operation.

        Parameters:
            operation_name: Name of the operation.
            kwargs: Parameters for the operation.
        Return:
            JSON string of the operation.
        """
        return json.dumps({"operation": operation_name, "parameters": kwargs})

    def generate_sequence(self, operations: List[str]) -> str:
        """Generate a JSON string representing a sequence of operations.

        Parameters:
            operations: List of JSON strings, each representing an operation.
        Return:
            JSON string of the entire sequence.
        """
        return json.dumps(operations)

    def translate_prompt_to_sequence(self, prompt: str) -> str:
        """Call chat gpt to convert the prompt from natual language from a sequence of operations in json format.

        Parameters:
            prompt: Natural language prompt describing desired operations.
        Return:
            JSON string representing the sequence of operations.
        """
        # Construct the natural language to operation sequence prompt
        instruction = f"""
            Please convert the following oral comamnd to machine readable operation json (list of json blobs)
            according to the API document.

            An example output would be:
            {{
                operations: [
                    {{
                        "operation": "move_single_servo",
                        "parameters": {{"id": "servo0", "angle": 60, "time": 500}}
                    }},
                    {{
                        "operation": "set_rgb_light",
                        "parameters": {{"R": 255, "G": 0, "B": 0}}
                    }},
                    {{
                        "operation": "move_single_servo",
                        "parameters": {{"id": "servo1", "angle": 90, "time": 500}}
                    }}
                ]
            }}
            
            The list of operations can be 1 ore many.

            Command:        
            ---
            {prompt}
            ---

            API Document:
            ---
            {self.api_document}
            ---
        """
        if self.verbose:
            self.logger.info(f"Instruction: {instruction}")
        response = self.gpt_client.chat.completions.create(
            model="gpt-3.5-turbo-1106",
            messages=[
                {
                    "role": "system",
                    "content": """
                        You are a translator that translate the natural language about robot arm operation
                        to a seuqnce of machine readable operations according to the API document specification
                    """,
                },
                {"role": "user", "content": instruction},
            ],
            response_format={"type": "json_object"},
            max_tokens=self.max_tokens,
            temperature=0.8,
        )
        content = response.choices[0].message.content
        operation_sequence_json_list = "[]" if not content else content.strip()
        if self.verbose:
            self.logger.info(
                f"Generated robot arm operations: {operation_sequence_json_list}"
            )
        return operation_sequence_json_list


class RobotArmControl(ABC):
    """Abstract class representing the control interface for a robotic arm."""

    @abstractmethod
    def _execute_operations(self, operations: List[Dict[str, Any]]) -> None:
        """
        Private method to be implemented by subclasses for executing operations.

        Parameters:
            operations: List of operation dictionaries.
        """
        pass

    def execute_operations(self, operations: str) -> None:
        """Execute a list of operations after validating them.

        Parameters:
            operations: JSON string representing a list of operations.
        """
        # Validate the operations JSON blob
        try:
            operations_obj = json.loads(operations)
            if "operations" in operations_obj:
                operations_list = operations_obj["operations"]
            if type(operations_list) is not list:
                raise ValueError(
                    f"Operations should be a json blob with a list, but got: {operations_list}"
                )
            if not isinstance(operations_list, list):
                raise ValueError("Operations should be a list.")

            # Validate whether operations is a json blob with a list.
            for idx, operation in enumerate(operations_list):
                if not isinstance(operation, dict):
                    raise ValueError(f"Operation {idx} should be a dictionary.")
                if "operation" not in operation:
                    raise ValueError(f"Operation {idx} should have an 'operation' key.")
                if "parameters" not in operation:
                    raise ValueError(f"Operation {idx} should have a 'parameters' key.")

            self._execute_operations(operations_list)
        except json.JSONDecodeError:
            raise ValueError("Invalid JSON format for operations.")


class SimulatedRobotArmControl(RobotArmControl):
    """
    Simulated control for a robotic arm, printing the operations instead of executing them.
    """

    def _execute_operations(self, operations: List[Dict]) -> None:
        for operation in operations:
            self.simulate_operation(operation)

    def simulate_operation(self, operation: Dict[str, Any]) -> str:
        """
        Simulate an individual operation.

        Parameters:
            operation: Dictionary representing the operation to simulate.
        """
        operation_name = operation["operation"]
        operation_params = operation["parameters"]
        print(
            Fore.BLUE
            + f"Executing {operation_name} with parameters: {operation_params}"
            + Fore.RESET
        )
        return ""


class RobotArmControlClient(RobotArmControl):
    """
    Real control for a robotic arm, sending operations as HTTP requests.
    """

    def __init__(self, endpoint_url: str = "http://0.0.0.0:5678/robot_command"):
        self.endpoint_url = endpoint_url

    def _execute_operations(self, operations: List[Dict[str, Any]]) -> None:
        """For now we only move one servo at a time.

        Args:
            operations (List[Dict[str, Any]]): _description_

        Raises:
            Exception: _description_
        """
        response = requests.post(self.endpoint_url, json={"operations": operations})
        if response.status_code != 200:
            raise Exception(f"Failed to execute operations: {response.text}")
        print(f"Operations executed successfully: {response.text}")
